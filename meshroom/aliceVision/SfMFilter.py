__version__ = "2.0"

from meshroom.core import desc
from meshroom.core.utils import VERBOSE_LEVEL

import ast
import operator
import re

# Check if input contains the txt string
def contains(input, txt):
    return (txt in input)

# Check if input match the regex string
def match(input, regex):
    return re.search(regex, input) is not None

# Return the lowercase version of the input string
def lower(input):
    return input.lower()

class SafeFilterParser:

    OPERATORS = {
        ast.Gt: operator.gt,
        ast.Lt: operator.lt,
        ast.GtE: operator.ge,
        ast.LtE: operator.le,
        ast.Eq: operator.eq,
        ast.NotEq: operator.ne,
    }

    functions = {
        "contains": contains,
        "match": match,
        "lower": lower,
    }
    
    def get_variables(self, expr_str):

        """Extract all variable names used in the expression"""
        tree = ast.parse(expr_str, mode='eval')
        variables = set()
        self._collect_variables(tree.body, variables)

        return list(variables)
    
    def _collect_variables(self, node, variables):
        """Recursively collect variable names from AST"""
        if isinstance(node, ast.Name):
            variables.add(node.id)
        
        elif isinstance(node, ast.Attribute):
            # For nested attributes like 'x.y', collect 'x'
            self._collect_variables(node.value, variables)
        
        # Traverse all child nodes
        for child in ast.iter_child_nodes(node):
            self._collect_variables(child, variables)
    
    def parse_expression(self, expr_str, obj,):

        """Safely evaluate expression against object"""
        tree = ast.parse(expr_str, mode='eval')
        return self._eval_node(tree.body, obj)
    
    def _eval_node(self, node, obj):
        if isinstance(node, ast.Compare):
            left = self._eval_node(node.left, obj)
            for op, comparator in zip(node.ops, node.comparators):
                right = self._eval_node(comparator, obj)
                op_func = self.OPERATORS.get(type(op))
                if not op_func:
                    raise ValueError(f"Unsupported operator: {type(op)}")
                if not op_func(left, right):
                    return False
                left = right
            return True
        elif isinstance(node, ast.Call):
            # Handle function calls
            if not isinstance(node.func, ast.Name):
                raise ValueError("Only simple function calls are supported")
            
            func_name = node.func.id
            if func_name not in self.functions:
                raise ValueError(f"Unknown function: {func_name}")
            
            # Evaluate arguments
            args = [self._eval_node(arg, obj) for arg in node.args]
            kwargs = {kw.arg: self._eval_node(kw.value, obj) for kw in node.keywords}
            
            # Call the function
            return self.functions[func_name](*args, **kwargs)

        elif isinstance(node, ast.Attribute):
            return getattr(obj, node.attr)
        elif isinstance(node, ast.Name):
            return getattr(obj, node.id)
        elif isinstance(node, ast.Constant):
            return node.value
        elif isinstance(node, ast.Num):  # Python < 3.8
            return node.n
        elif isinstance(node, ast.Str):  # Python < 3.8
            return node.s
        else:
            raise ValueError(f"Unsupported node type: {type(node)}")
    
    def filter(self, elements, expression):
        return [e for e in elements if self.parse_expression(expression, e)]

class SfMFilter(desc.Node):
    """
    Filtering views from an input SfMData given a validation expression.
    For each view, the expression is evaluated, and the view is selected if the expression returns True.
    Two SfMData are generated, one for the selected views, one for the non selected views.
    The result SfMData are kept clean (e.g. observations from a removed view are removed).

    Allowed binary operators are (>,<,<=,>=,==,!=).
    
    Allowed functions are :
        - lower(str) : return the lowercase version of the string
        - contains(str, substr) : return true if str contains substr
        - match(str, regex) : return true if the regex pattern is found in str

    Allowed variables per view are:
        - path : the view image path
        - id : the view id
        - frameId : the view's frame id
        - observedLandmarks : the count of observed landmarks
    """

    size = desc.DynamicNodeSize("inputFile")
    category = "Utils"
    inputs = [
        desc.File(
            name="inputFile",
            label="Input File",
            description="SfMData file.",
            value="",
        ),
        desc.StringParam(
            name="expression",
            label="View selection expression",
            description="See node documentation for details",
            value="True",
        ),
        desc.ChoiceParam(
            name="verboseLevel",
            label="Verbose Level",
            description="Verbosity level (fatal, error, warning, info, debug, trace).",
            values=VERBOSE_LEVEL,
            value="info",
        ),
    ]

    outputs = [
        desc.File(
            name="outputSfMData_selected",
            label="Selected SfMData",
            description="Output SfMData file containing selected views.",
            value="{nodeCacheFolder}/selectedSfmData.abc",
        ),
        desc.File(
            name="outputSfMData_unselected",
            label="Unselected SfMData",
            description="Output SfMData file containing remaining views.",
            value="{nodeCacheFolder}/unselectedSfmData.abc",
        ),
    ]

    def processChunk(self, chunk):
        
        from pyalicevision import sfmData as avsfmdata
        from pyalicevision import sfmDataIO as avsfmdataio

        chunk.logManager.start(chunk.node.verboseLevel.value)
        chunk.logger.info(f"Open input file {chunk.node.inputFile.value}")

        expression = chunk.node.expression.value
        if expression == "":
            expression = "True"

        dataAV = avsfmdata.SfMData()
        ret = avsfmdataio.load(dataAV, chunk.node.inputFile.value, avsfmdataio.ALL)
        if not ret:
            chunk.logger.error("Cannot open input")
            chunk.logManager.end()
            raise RuntimeError()

        
        class ViewInfo:
            def __init__(self, id, frameId, path, observedLandmarks):
                self.id = id
                self.path = path
                self.frameId = frameId
                self.observedLandmarks = observedLandmarks

        parser = SafeFilterParser()

        # Retrieve all variables
        variables = []
        try:
            variables = parser.get_variables(expression)
        except Exception as e:
            chunk.logger.error("Invalid expression.")
            raise RuntimeError()

        chunk.logger.info("Variables used :")
        for variable in variables:
            chunk.logger.info(f"- {variable}")

        #Do we need to compute observed landmarks
        observedLandmarks = {key: 0 for key in dataAV.getViews().keys()}
        if "observedLandmarks" in variables:
            chunk.logger.info("observedLandmarks have to be precomputed.")
            landmarks = dataAV.getLandmarks()
            for lid, landmark in landmarks.items():
                for vid in landmark.getMapObservations().keys():
                    observedLandmarks[vid] = observedLandmarks[vid] + 1


        # Prepare the fields used by the parser
        # This contains the info needed per view
        viewInfos = []
        views = dataAV.getViews()
        for id, v in views.items():

            #retrieve observed landmarks
            observed = observedLandmarks[id]
            
            #Add view info
            v = ViewInfo(id, v.getFrameId(), v.getImageInfo().getImagePath(), observed)
            viewInfos.append(v)

        result = parser.filter(viewInfos, expression)

        selectedViews = avsfmdata.Views()
        unselectedViews = avsfmdata.Views()
        
        #Now we filter out the views using the parser result
        for vid, v in views.items():
            found = False
            #Is it kept by the filter ?
            
            for item in result:
                if vid == item.id:
                    found = True
                    break

            if found:
                selectedViews[vid] = v
            else:
                unselectedViews[vid] = v

        chunk.logger.info(f"{len(selectedViews)}/{len(views)} views selected.")

        # if needed, output the selected views sfmData
        if chunk.node.outputSfMData_selected.value != "":
            output = avsfmdata.SfMData(dataAV, True)
            outputViews = output.getViews()
            outputViews.clone(selectedViews)
            output.repair()

            chunk.logger.info(f"Saving selected sfmData to {chunk.node.outputSfMData_selected.value}.")
            avsfmdataio.save(output, chunk.node.outputSfMData_selected.value, avsfmdataio.ALL)

        # if needed, output the unselected views sfmData
        if chunk.node.outputSfMData_unselected.value != "":
            output = avsfmdata.SfMData(dataAV, True)
            outputViews = output.getViews()
            outputViews.clone(unselectedViews)
            output.repair()

            chunk.logger.info(f"Saving unselected sfmData to {chunk.node.outputSfMData_unselected.value}.")
            avsfmdataio.save(output, chunk.node.outputSfMData_unselected.value, avsfmdataio.ALL)


        chunk.logManager.end()