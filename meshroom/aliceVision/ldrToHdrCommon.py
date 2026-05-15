from meshroom.core.attribute import Attribute
from meshroom.core.node import Node
from meshroom.core.desc.validators import AttributeValidator, success, error


class ImageCountShouldBeAMultipleOfBracketNumber(AttributeValidator):

    def __call__(self, node: Node, attribute: Attribute):

        if node.userNbBrackets.value == 0:
            return success()

        cameraInitOutput = node.input.inputRootLink

        # The number of brackets has been manually forced: check whether it is valid or not
        if cameraInitOutput and cameraInitOutput.node and cameraInitOutput.node.hasAttribute("viewpoints"):
            viewpoints = cameraInitOutput.node.viewpoints.value
            # The number of brackets should be a multiple of the number of input images
            if (len(viewpoints) % node.userNbBrackets.value != 0):
                return error(
                        "The set number of brackets is not a multiple of the number of input images.",
                        "Errors will occur during the computation."
                    )
            else:
                return success()

        return success()
