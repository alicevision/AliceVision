#pragma once

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/boykov_kolmogorov_max_flow.hpp>

#include <aliceVision/image/all.hpp>

#include "distance.hpp"
#include "boundingBox.hpp"
#include "imageOps.hpp"
#include "seams.hpp"

namespace aliceVision
{

/**
 * @brief Maxflow computation based on a standard Adjacency List graph reprensentation.
 *
 * @see MaxFlow_CSR which use less memory.
 */
class MaxFlow_AdjList
{
public:
    using NodeType = int;
    using ValueType = float;

    using Traits = boost::adjacency_list_traits<boost::vecS, // OutEdgeListS
                                                boost::vecS, // VertexListS
                                                boost::directedS,
                                                boost::vecS // EdgeListS
                                                >;
    using edge_descriptor = typename Traits::edge_descriptor;
    using vertex_descriptor = typename Traits::vertex_descriptor;
    using vertex_size_type = typename Traits::vertices_size_type;
    struct Edge
    {
        ValueType capacity{};
        ValueType residual{};
        edge_descriptor reverse;
    };
    using Graph = boost::adjacency_list<boost::vecS, // OutEdgeListS
                                        boost::vecS, // VertexListS
                                        boost::directedS,
                                        boost::no_property, // VertexProperty
                                        Edge,               // EdgeProperty
                                        boost::no_property, // GraphProperty
                                        boost::vecS         // EdgeListS
                                        >;
    using VertexIterator = typename boost::graph_traits<Graph>::vertex_iterator;

public:
    explicit MaxFlow_AdjList(size_t numNodes)
        : _graph(numNodes + 2)
        , _S(NodeType(numNodes))
        , _T(NodeType(numNodes + 1))
    {
        VertexIterator vi, vi_end;
        for(boost::tie(vi, vi_end) = vertices(_graph); vi != vi_end; ++vi)
        {
            _graph.m_vertices[*vi].m_out_edges.reserve(9);
        }
        _graph.m_vertices[numNodes].m_out_edges.reserve(numNodes);
        _graph.m_vertices[numNodes + 1].m_out_edges.reserve(numNodes);
    }

    inline void addNodeToSource(NodeType n, ValueType source)
    {
        assert(source >= 0);

        edge_descriptor edge(boost::add_edge(_S, n, _graph).first);
        edge_descriptor reverseEdge(boost::add_edge(n, _S, _graph).first);

        _graph[edge].capacity = source;
        _graph[edge].reverse = reverseEdge;
        _graph[reverseEdge].reverse = edge;
        _graph[reverseEdge].capacity = source;
    }

    inline void addNodeToSink(NodeType n, ValueType sink)
    {
        assert(sink >= 0);

        edge_descriptor edge(boost::add_edge(_T, n, _graph).first);
        edge_descriptor reverseEdge(boost::add_edge(n, _T, _graph).first);

        _graph[edge].capacity = sink;
        _graph[edge].reverse = reverseEdge;
        _graph[reverseEdge].reverse = edge;
        _graph[reverseEdge].capacity = sink;
    }

    inline void addEdge(NodeType n1, NodeType n2, ValueType capacity, ValueType reverseCapacity)
    {
        assert(capacity >= 0 && reverseCapacity >= 0);

        edge_descriptor edge(boost::add_edge(n1, n2, _graph).first);
        edge_descriptor reverseEdge(boost::add_edge(n2, n1, _graph).first);
        _graph[edge].capacity = capacity;
        _graph[edge].reverse = reverseEdge;

        _graph[reverseEdge].capacity = reverseCapacity;
        _graph[reverseEdge].reverse = edge;
    }

    void printStats() const;
    void printColorStats() const;

    inline ValueType compute()
    {
        vertex_size_type nbVertices(boost::num_vertices(_graph));
        _color.resize(nbVertices, boost::white_color);
        std::vector<edge_descriptor> pred(nbVertices);
        std::vector<vertex_size_type> dist(nbVertices);

        ValueType v = boost::boykov_kolmogorov_max_flow(_graph, boost::get(&Edge::capacity, _graph),
                                                        boost::get(&Edge::residual, _graph),
                                                        boost::get(&Edge::reverse, _graph), &pred[0], &_color[0],
                                                        &dist[0], boost::get(boost::vertex_index, _graph), _S, _T);

        return v;
    }

    /// is empty
    inline bool isSource(NodeType n) const { return (_color[n] == boost::black_color); }
    /// is full
    inline bool isTarget(NodeType n) const { return (_color[n] == boost::white_color); }

protected:
    Graph _graph;
    std::vector<boost::default_color_type> _color;
    const NodeType _S; //< emptyness
    const NodeType _T; //< fullness
};


bool computeSeamsMap(image::Image<unsigned char>& seams, const image::Image<IndexT>& labels);

class GraphcutSeams
{
public:
    struct InputData 
    {
        IndexT id;
        BoundingBox rect;
        image::Image<image::RGBfColor> color;
        image::Image<unsigned char> mask;
    };

    using IndexedColor = std::pair<IndexT, image::RGBfColor>;
    using PixelInfo = std::vector<IndexedColor>;
    

public:
    GraphcutSeams(size_t outputWidth, size_t outputHeight)
        : _outputWidth(outputWidth)
        , _outputHeight(outputHeight)
        , _maximal_distance_change(outputWidth + outputHeight)
        , _labels(outputWidth, outputHeight, true, UndefinedIndexT)
    {
    }

    virtual ~GraphcutSeams() = default;

    std::pair<IndexT, image::RGBfColor> findIndex(const PixelInfo & pix, IndexT id) 
    {
        for (int i = 0; i  < pix.size(); i++)
        {
            if (pix[i].first == id)
            {
                return pix[i];
            }
        }

        return std::make_pair(UndefinedIndexT, image::RGBfColor(0.0f));
    }

    bool existIndex(const PixelInfo & pix, IndexT id) 
    {
        for (int i = 0; i  < pix.size(); i++)
        {
            if (pix[i].first == id)
            {
                return true;
            }
        }

        return false;
    }

    bool setOriginalLabels(const image::Image<IndexT> & existing_labels)
    {   
        _labels = existing_labels;

        return true;
    }

    bool append(const image::Image<image::RGBfColor>& input, const image::Image<unsigned char>& inputMask, IndexT currentIndex, size_t offset_x, size_t offset_y)
    {

        if(inputMask.Width() != input.Width())
        {
            return false;
        }

        if(inputMask.Height() != input.Height())
        {
            return false;
        }

        InputData data;

        data.id = currentIndex;
        data.color = input;
        data.mask = inputMask;
        data.rect.width = input.Width();
        data.rect.height = input.Height();
        data.rect.left = offset_x;
        data.rect.top = offset_y;

        _inputs[currentIndex] = data;


        return true;
    }

    void setMaximalDistance(int dist) 
    { 
        _maximal_distance_change = dist; 
    }

    bool createInputOverlappingObservations(image::Image<PixelInfo> & graphCutInput, const BoundingBox & interestBbox)
    {
        for (auto  & otherInput : _inputs)
        {
            BoundingBox otherBbox = otherInput.second.rect;
            BoundingBox otherBboxLoop = otherInput.second.rect;
            otherBboxLoop.left = otherBbox.left - _outputWidth;
            BoundingBox otherBboxLoopRight = otherInput.second.rect;
            otherBboxLoopRight.left = otherBbox.left + _outputWidth;

            BoundingBox otherInputBbox = otherBbox;
            otherInputBbox.left = 0;
            otherInputBbox.top = 0;

            BoundingBox intersection = interestBbox.intersectionWith(otherBbox);
            BoundingBox intersectionLoop = interestBbox.intersectionWith(otherBboxLoop);
            BoundingBox intersectionLoopRight = interestBbox.intersectionWith(otherBboxLoopRight);
            if (intersection.isEmpty() && intersectionLoop.isEmpty() && intersectionLoopRight.isEmpty())
            {
                continue;
            }
            
            image::Image<image::RGBfColor> otherColor(otherInputBbox.width, otherInputBbox.height);
            otherColor.block(otherInputBbox.top, otherInputBbox.left, otherInputBbox.height, otherInputBbox.width) = otherInput.second.color.block(otherInputBbox.top, otherInputBbox.left, otherInputBbox.height, otherInputBbox.width);

            image::Image<unsigned char> otherMask(otherInputBbox.width, otherInputBbox.height);
            otherMask.block(otherInputBbox.top, otherInputBbox.left, otherInputBbox.height, otherInputBbox.width) = otherInput.second.mask.block(otherInputBbox.top, otherInputBbox.left, otherInputBbox.height, otherInputBbox.width);
 
            if (!intersection.isEmpty())
            {        
                BoundingBox interestOther = intersection;
                interestOther.left -= otherBbox.left;
                interestOther.top -= otherBbox.top;

                BoundingBox interestThis = intersection;
                interestThis.left -= interestBbox.left;
                interestThis.top -= interestBbox.top;

                for (int y = 0; y < intersection.height; y++)
                {
                    int y_other = interestOther.top + y;
                    int y_current = interestThis.top + y;

                    for (int x = 0; x < intersection.width; x++)
                    {
                        int x_other = interestOther.left + x;
                        int x_current = interestThis.left + x;
                        

                        if (!otherMask(y_other, x_other))
                        {
                            continue;
                        }
                      
                        PixelInfo & pix = graphCutInput(y_current, x_current);
                        pix.push_back(std::make_pair(otherInput.first, otherColor(y_other, x_other)));
                    }
                }
            }

            if (!intersectionLoop.isEmpty())
            {
                BoundingBox interestOther = intersectionLoop;
                interestOther.left -= otherBboxLoop.left;
                interestOther.top -= otherBboxLoop.top;

                BoundingBox interestThis = intersectionLoop;
                interestThis.left -= interestBbox.left;
                interestThis.top -= interestBbox.top;

                
                for (int y = 0; y < intersectionLoop.height; y++)
                {
                    int y_other = interestOther.top + y;
                    int y_current = interestThis.top + y;

                    for (int x = 0; x < intersectionLoop.width; x++)
                    {
                        int x_other = interestOther.left + x;
                        int x_current = interestThis.left + x;

                        if (!otherMask(y_other, x_other))
                        {
                            continue;
                        }

                        PixelInfo & pix = graphCutInput(y_current, x_current);
                        pix.push_back(std::make_pair(otherInput.first, otherColor(y_other, x_other)));
                    }
                }
            }

            if (!intersectionLoopRight.isEmpty())
            {
                BoundingBox interestOther = intersectionLoopRight;
                interestOther.left -= otherBboxLoopRight.left;
                interestOther.top -= otherBboxLoopRight.top;

                BoundingBox interestThis = intersectionLoopRight;
                interestThis.left -= interestBbox.left;
                interestThis.top -= interestBbox.top;

                
                for (int y = 0; y < intersectionLoopRight.height; y++)
                {
                    int y_other = interestOther.top + y;
                    int y_current = interestThis.top + y;

                    for (int x = 0; x < intersectionLoopRight.width; x++)
                    {
                        int x_other = interestOther.left + x;
                        int x_current = interestThis.left + x;

                        if (!otherMask(y_other, x_other))
                        {
                            continue;
                        }

                        PixelInfo & pix = graphCutInput(y_current, x_current);
                        pix.push_back(std::make_pair(otherInput.first, otherColor(y_other, x_other)));
                    }
                }
            }
        }

        return true;
    }

    bool fixUpscaling(image::Image<IndexT> & labels, const image::Image<PixelInfo> & graphCutInput) 
    {
        //Because of upscaling, some labels may be incorrect
        //Some pixels may be affected to labels they don't see.
        for (int y = 0; y < graphCutInput.Height(); y++) 
        {
            for (int x = 0; x < graphCutInput.Width(); x++) 
            {
                IndexT label = labels(y, x);

                if (label == UndefinedIndexT)
                {
                    continue;
                }

                // If there is no input for this pixel, we can't do anything
                const PixelInfo & pix = graphCutInput(y, x);
                
                // Check if the input associated to this label is seen by this pixel
                auto it = existIndex(pix, label);
                if (it)
                {
                    continue;
                }

                // Look for another label in the neighboorhood which is seen by this pixel
                bool modified = false;
                bool hadUndefined = false;
                for (int l = -1; l <= 1; l++) 
                {
                    int ny = y + l;
                    if (ny < 0 || ny >= labels.Height()) 
                    {
                        continue;
                    } 

                    for (int c = -1; c <= 1; c++)
                    {
                        int nx = x + c;
                        if (nx < 0 || nx >= labels.Width()) 
                        {
                            continue;
                        }

                        IndexT otherLabel = labels(ny, nx);                     
                        if (otherLabel == label)
                        {
                            continue;
                        }

                        if (otherLabel == UndefinedIndexT) 
                        {
                            hadUndefined = true;
                            continue;
                        }

                        // Check that this other label is seen by our pixel
                        const PixelInfo & pixOther = graphCutInput(ny, nx); 
                        auto itOther = existIndex(pixOther, otherLabel);
                        if (!itOther)
                        {
                            continue;
                        }

                        auto it = existIndex(pix, otherLabel);
                        if (it)
                        {
                            labels(y, x) = otherLabel;
                            modified = true;
                        }
                    }
                }

                if (!modified) 
                {
                    labels(y, x) = UndefinedIndexT;
                }
            }
        }

        return true;
    }

    bool computeInputDistanceMap(image::Image<int> & distanceMap, const image::Image<IndexT> & localLabels, IndexT inputId)
    {
        image::Image<IndexT> binarizedWorld(localLabels.Width(), localLabels.Height());

        for(int y = 0; y < localLabels.Height(); y++)
        {
            for(int x = 0; x < localLabels.Width(); x++)
            {
                IndexT label = localLabels(y, x);

                if(label == inputId)
                {
                    binarizedWorld(y, x) = 1;
                }
                else
                {
                    binarizedWorld(y, x) = 0;
                }
            }
        }

        image::Image<unsigned char> seams(localLabels.Width(), localLabels.Height());
        if(!computeSeamsMap(seams, binarizedWorld))
        {
            return false;
        }

        if(!computeDistanceMap(distanceMap, seams))
        {
            return false;
        }

        return true;
    }

    bool processInput(double & newCost, InputData & input)
    {       

        //Get bounding box of input in panorama
        //Dilate to have some pixels outside of the input
        BoundingBox localBbox = input.rect.dilate(3);
        localBbox.clampLeft();
        localBbox.clampTop();
        localBbox.clampBottom(_labels.Height() - 1);
        
        //Output must keep a margin also
        BoundingBox outputBbox = input.rect;
        outputBbox.left = input.rect.left - localBbox.left;
        outputBbox.top = input.rect.top - localBbox.top;

        //Extract labels for the ROI
        image::Image<IndexT> localLabels(localBbox.width, localBbox.height);
        if (!loopyImageExtract(localLabels, _labels, localBbox)) 
        {
            return false;
        }   

        //Build the input
        image::Image<PixelInfo> graphCutInput(localBbox.width, localBbox.height, true);
        if (!createInputOverlappingObservations(graphCutInput, localBbox))
        {
            return false;
        }

        // Fix upscaling induced bad labeling
        if (!fixUpscaling(localLabels, graphCutInput))
        {
            return false;
        }

        // Backup update for upscaling 
        BoundingBox inputBb = localBbox;
        inputBb.left = 0;
        inputBb.top = 0;
        if (!loopyImageAssign(_labels, localLabels, localBbox, inputBb)) 
        {
            return false;
        }

        // Compute distance map to borders of the input seams
        image::Image<int> distanceMap(localLabels.Width(), localLabels.Height());
        if (!computeInputDistanceMap(distanceMap, localLabels, input.id))
        {
            return false;
        }

        //Remove pixels too far from seams
        for (int i = 0; i < graphCutInput.Height(); i++) 
        {
            for (int j = 0; j < graphCutInput.Width(); j++)
            {
                float d2 = float(distanceMap(i, j));
                float d = sqrt(d2);

                if (d > _maximal_distance_change + 1.0f)
                {
                    graphCutInput(i, j).clear();
                }
            } 
        }
        
        double oldCost = cost(localLabels, graphCutInput, input.id);
        if (!alphaExpansion(localLabels, distanceMap, graphCutInput, input.id)) 
        {
            return false;
        }
        newCost = cost(localLabels, graphCutInput, input.id);


        if (newCost < oldCost)
        {   
            BoundingBox inputBb = localBbox;
            inputBb.left = 0;
            inputBb.top = 0;
            
            if (!loopyImageAssign(_labels, localLabels, localBbox, inputBb)) 
            {
                return false;
            }
        }
        else 
        {
            newCost = oldCost;  
        }

        return true;
    }

    bool process()
    {
        std::map<IndexT, double> costs;
        for (auto & info : _inputs)
        {
            costs[info.first] = std::numeric_limits<double>::max();
        }

        for (int i = 0; i < 10; i++)
        {
            ALICEVISION_LOG_INFO("GraphCut processing iteration #" << i);

            // For each possible label, try to extends its domination on the label's world
            bool hasChange = false;

            int pos = 0;
            for (auto & info : _inputs)
            {   
                double cost;
                if (!processInput(cost, info.second)) 
                {
                    return false;
                }
                

                if (costs[info.first] != cost)
                {
                    costs[info.first] = cost;
                    hasChange = true;
                }
            }

            if (!hasChange)
            {
                break;
            }
        }

        return true;
    }

    double cost(const image::Image<IndexT> localLabels, const image::Image<PixelInfo> & input, IndexT currentLabel)
    {
        double cost = 0.0;

        for (int y = 0; y < input.Height() - 1; y++)
        {
            for(int x = 0; x < input.Width() - 1; x++) 
            {
                int xp = x + 1;
                int yp = y + 1;

                IndexT label = localLabels(y, x);
                IndexT labelx = localLabels(y, xp);
                IndexT labely = localLabels(yp, x);

                if(label == UndefinedIndexT)
                    continue;
                if(labelx == UndefinedIndexT)
                    continue;
                if(labely == UndefinedIndexT)
                    continue;

                image::RGBfColor CColorLC;
                image::RGBfColor CColorLX;
                image::RGBfColor CColorLY;
                image::RGBfColor XColorLC;
                image::RGBfColor XColorLX;
                image::RGBfColor YColorLC;
                image::RGBfColor YColorLY;
                bool hasYLC = false;
                bool hasYLY = false;
                bool hasXLC = false;
                bool hasXLX = false;
                bool hasCLC = false;
                bool hasCLX = false;
                bool hasCLY = false;


                auto it = findIndex(input(y, x), label);
                if (it.first != UndefinedIndexT)
                {
                    hasCLC = true;
                    CColorLC = it.second;
                }

                it = findIndex(input(y, x), labelx);
                if (it.first != UndefinedIndexT)
                {
                    hasCLX = true;
                    CColorLX = it.second;
                }

                it = findIndex(input(y, x), labely);
                if (it.first != UndefinedIndexT)
                {
                    hasCLY = true;
                    CColorLY = it.second;
                }

                it = findIndex(input(y, xp), label);
                if (it.first != UndefinedIndexT)
                {
                    hasXLC = true;
                    XColorLC = it.second;
                }

                it = findIndex(input(y, xp), labelx);
                if (it.first != UndefinedIndexT)
                {
                    hasXLX = true;
                    XColorLX = it.second;
                }

                it = findIndex(input(yp, x), label);
                if (it.first != UndefinedIndexT)
                {
                    hasYLC = true;
                    YColorLC = it.second;
                }

                it = findIndex(input(yp, x), labely);
                if (it.first != UndefinedIndexT)
                {
                    hasYLY = true;
                    YColorLY = it.second;
                }

                if(!hasCLC || !hasXLX || !hasYLY)
                {
                    continue;
                }

                if(!hasCLX)
                {
                    CColorLX = CColorLC;
                }

                if(!hasCLY)
                {
                    CColorLY = CColorLC;
                }

                if(!hasXLC)
                {
                    XColorLC = XColorLX;
                }

                if(!hasYLC)
                {
                    YColorLC = YColorLY;
                }

                double c1 = (CColorLC - CColorLX).norm();
                double c2 = (CColorLC - CColorLY).norm();
                double c3 = (XColorLC - XColorLX).norm();
                double c4 = (YColorLC - YColorLY).norm();

                c1 = std::min(2.0, c1);
                c2 = std::min(2.0, c2);
                c3 = std::min(2.0, c3);
                c4 = std::min(2.0, c4);

                cost += c1 + c2 + c3 + c4;
            }
        }

        return cost;
    }

    bool alphaExpansion(image::Image<IndexT> & labels, const image::Image<int> & distanceMap, const image::Image<PixelInfo> & input, IndexT currentLabel)
    {
        image::Image<unsigned char> mask(labels.Width(), labels.Height(), true, 0);
        image::Image<int> ids(labels.Width(), labels.Height(), true, -1);
        image::Image<image::RGBfColor> color_label(labels.Width(), labels.Height(), true, image::RGBfColor(0.0f, 0.0f, 0.0f));
        image::Image<image::RGBfColor> color_other(labels.Width(), labels.Height(), true, image::RGBfColor(0.0f, 0.0f, 0.0f));


        for (int y = 0; y < labels.Height(); y++) 
        {
            for (int x = 0; x < labels.Width(); x++)
            {
                IndexT label = labels(y, x);

                float dist = sqrt(float(distanceMap(y, x)));

                image::RGBfColor currentColor;
                image::RGBfColor otherColor;

                auto it = findIndex(input(y, x), currentLabel);
                if (it.first != UndefinedIndexT)
                {
                    currentColor = it.second;
                    mask(y, x) = 1;
                }

                if (label != currentLabel)
                {
                    auto it = findIndex(input(y, x), label);
                    if (it.first != UndefinedIndexT)
                    {
                        otherColor = it.second;

                        if (dist > _maximal_distance_change)
                        {
                            mask(y, x) = 2;
                        }
                        else 
                        {
                            mask(y, x) |= 2;
                        }
                    }
                }

                // If the pixel may be a new kingdom for alpha 
                if(mask(y, x) == 1)
                {
                    color_label(y, x) = currentColor;
                    color_other(y, x) = currentColor;
                }
                else if(mask(y, x) == 2)
                {
                    color_label(y, x) = otherColor;
                    color_other(y, x) = otherColor;
                }
                else if(mask(y, x) == 3)
                {
                    color_label(y, x) = currentColor;
                    color_other(y, x) = otherColor;
                }
            }
        }     

        // The rectangle is a grid.
        // However we want to ignore a lot of pixel.
        // Let's create an index per valid pixels for graph cut reference
        int count = 0;
        for(int y = 0; y < labels.Height(); y++)
        {
            for(int x = 0; x < labels.Width(); x++)
            {
                if(mask(y, x) == 0)
                {
                    continue;
                }

                ids(y, x) = count;
                count++;
            }
        }  

        //Create graph
        MaxFlow_AdjList gc(count);
        size_t countValid = 0;

        for(int y = 0; y < labels.Height(); y++)
        {
            for(int x = 0; x < labels.Width(); x++)
            {

                // If this pixel is not valid, ignore 
                if(mask(y, x) == 0)
                {
                    continue;
                }

                // Get this pixel ID 
                int node_id = ids(y, x);

                int ym1 = std::max(y - 1, 0);
                int xm1 = std::max(x - 1, 0);
                int yp1 = std::min(y + 1, labels.Height() - 1);
                int xp1 = std::min(x + 1, labels.Width() - 1);

                if(mask(y, x) == 1)
                {
                    // Only add nodes close to borders 
                    if(mask(ym1, xm1) == 1 && mask(ym1, x) == 1 && mask(ym1, xp1) == 1 &&
                       mask(y, xm1) == 1 && mask(y, xp1) == 1 && 
                       mask(yp1, xm1) == 1 && mask(yp1, x) == 1 && mask(yp1, xp1) == 1)
                    {
                        continue;
                    }

                    
                    //This pixel is only seen by alpha.
                    //Enforce its domination by stating that removing this pixel
                    //from alpha territoy is infinitly costly (impossible).
                    gc.addNodeToSource(node_id, 100000);
                }
                else if(mask(y, x) == 2)
                {
                    // Only add nodes close to borders
                    if(mask(ym1, xm1) == 2 && mask(ym1, x) == 2 && mask(ym1, xp1) == 2 &&
                       mask(y, xm1) == 2 && mask(y, xp1) == 2 && 
                       mask(yp1, xm1) == 2 && mask(yp1, x) == 2 && mask(yp1, xp1) == 2)
                    {
                        continue;
                    }

                    //This pixel is only seen by an ennemy.
                    //Enforce its domination by stating that removing this pixel
                    //from ennemy territory is infinitly costly (impossible).
                    gc.addNodeToSink(node_id, 100000);
                }
                else if(mask(y, x) == 3)
                {

                    // This pixel is seen by both alpha and enemies but is owned by ennemy.
                    // Make sure that changing node owner will have no direct cost.
                    // Connect it to both alpha and ennemy for the moment
                    // (Graph cut will not allow a pixel to have both owners at the end).
                    gc.addNodeToSource(node_id, 0);
                    gc.addNodeToSink(node_id, 0);
                    countValid++;
                }
            }
        }


        if(countValid == 0)
        {
            // We have no possibility for territory expansion 
            // let's exit
            return true;
        }


        // Loop over alpha bounding box.
        // Let's define the transition cost.
        // When two neighboor pixels have different labels, there is a seam (border) cost.
        // Graph cut will try to make sure the territory will have a minimal border cost

        for(int y = 0; y < labels.Height(); y++)
        {
            for(int x = 0; x < labels.Width(); x++)
            {

                if(mask(y, x) == 0)
                {
                    continue;
                }

                int node_id = ids(y, x);

                // Make sure it is possible to estimate this horizontal border
                if(y < mask.Height() - 1)
                {
                    // Make sure the other pixel is owned by someone
                    if(mask(y + 1, x))
                    {

                        int other_node_id = ids(y + 1, x);
                        float w = 1000;

                        if(((mask(y, x) & 1) && (mask(y + 1, x) & 2)) || ((mask(y, x) & 2) && (mask(y + 1, x) & 1)))
                        {
                            float d1 = (color_label(y, x) - color_other(y, x)).norm();
                            float d2 = (color_label(y + 1, x) - color_other(y + 1, x)).norm();

                            d1 = std::min(2.0f, d1);
                            d2 = std::min(2.0f, d2);

                            w = (d1 + d2) * 100.0 + 1.0;
                        }

                        gc.addEdge(node_id, other_node_id, w, w);
                    }
                }

                if(x < mask.Width() - 1)
                {

                    if(mask(y, x + 1))
                    {

                        int other_node_id = ids(y, x + 1);
                        float w = 1000;

                        if(((mask(y, x) & 1) && (mask(y, x + 1) & 2)) || ((mask(y, x) & 2) && (mask(y, x + 1) & 1)))
                        {
                            float d1 = (color_label(y, x) - color_other(y, x)).norm();
                            float d2 = (color_label(y, x + 1) - color_other(y, x + 1)).norm();

                            d1 = std::min(2.0f, d1);
                            d2 = std::min(2.0f, d2);

                            w = (d1 + d2) * 100.0 + 1.0;
                        }

                        gc.addEdge(node_id, other_node_id, w, w);
                    }
                }
            }
        }

        gc.compute();

        int changeCount = 0;
        for(int y = 0; y < labels.Height(); y++)
        {
            for(int x = 0; x < labels.Width(); x++)
            {
                IndexT label = labels(y, x);
                int id = ids(y, x);

                if(gc.isSource(id))
                {

                    if(label != currentLabel)
                    {
                        changeCount++;
                    }

                    labels(y, x) = currentLabel;
                }
            }
        }

        return true;
    }

    image::Image<IndexT> & getLabels() 
    { 
        return _labels; 
    }

private:

    std::map<IndexT, InputData> _inputs;

    int _outputWidth;
    int _outputHeight;
    size_t _maximal_distance_change;
    image::Image<IndexT> _labels;
};

} // namespace aliceVision
