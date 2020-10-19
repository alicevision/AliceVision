#pragma once

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/boykov_kolmogorov_max_flow.hpp>

#include <aliceVision/image/all.hpp>

#include "distance.hpp"
#include "boundingBox.hpp"
#include "imageOps.hpp"

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

bool computeSeamsMap(image::Image<unsigned char>& seams, const image::Image<IndexT>& labels)
{

    if(seams.size() != labels.size())
    {
        return false;
    }

    seams.fill(0);

    for(int j = 1; j < labels.Width() - 1; j++)
    {
        IndexT label = labels(0, j);
        IndexT same = true;

        same &= (labels(0, j - 1) == label);
        same &= (labels(0, j + 1) == label);
        same &= (labels(1, j - 1) == label);
        same &= (labels(1, j) == label);
        same &= (labels(1, j + 1) == label);

        if(same)
        {
            continue;
        }

        seams(0, j) = 255;
    }

    int lastrow = labels.Height() - 1;
    for(int j = 1; j < labels.Width() - 1; j++)
    {
        IndexT label = labels(lastrow, j);
        IndexT same = true;

        same &= (labels(lastrow - 1, j - 1) == label);
        same &= (labels(lastrow - 1, j + 1) == label);
        same &= (labels(lastrow, j - 1) == label);
        same &= (labels(lastrow, j) == label);
        same &= (labels(lastrow, j + 1) == label);

        if(same)
        {
            continue;
        }

        seams(lastrow, j) = 255;
    }

    for(int i = 1; i < labels.Height() - 1; i++)
    {

        for(int j = 1; j < labels.Width() - 1; j++)
        {

            IndexT label = labels(i, j);
            IndexT same = true;

            same &= (labels(i - 1, j - 1) == label);
            same &= (labels(i - 1, j) == label);
            same &= (labels(i - 1, j + 1) == label);
            same &= (labels(i, j - 1) == label);
            same &= (labels(i, j + 1) == label);
            same &= (labels(i + 1, j - 1) == label);
            same &= (labels(i + 1, j) == label);
            same &= (labels(i + 1, j + 1) == label);

            if(same)
            {
                continue;
            }

            seams(i, j) = 255;
        }
    }

    return true;
}


class GraphcutSeams
{
public:
    struct InputData 
    {
        IndexT id;
        BoundingBox rect;
        CachedImage<image::RGBfColor> color;
        CachedImage<unsigned char> mask;
    };

    using PixelInfo = std::map<IndexT, image::RGBfColor>;
    

public:
    GraphcutSeams(size_t outputWidth, size_t outputHeight)
        : _outputWidth(outputWidth), _outputHeight(outputHeight)
        , _maximal_distance_change(outputWidth + outputHeight)
    {
    }

    virtual ~GraphcutSeams() = default;

    bool setOriginalLabels(CachedImage<IndexT> & existing_labels)
    {   
        if (!_labels.deepCopy(existing_labels)) 
        {
            return false;
        }
        
        return true;
    }

    bool initialize(image::TileCacheManager::shared_ptr & cacheManager) 
    {
        if(!_labels.createImage(cacheManager, _outputWidth, _outputHeight))
        {
            return false;
        }

        return true;
    }

    bool append(const CachedImage<image::RGBfColor>& input, const CachedImage<unsigned char>& inputMask, IndexT currentIndex, size_t offset_x, size_t offset_y)
    {

        if(inputMask.getWidth() != input.getWidth())
        {
            return false;
        }

        if(inputMask.getHeight() != input.getHeight())
        {
            return false;
        }

        InputData data;
        data.id = currentIndex;
        data.color = input;
        data.mask = inputMask;
        data.rect.width = input.getWidth();
        data.rect.height = input.getHeight();
        data.rect.left = offset_x;
        data.rect.top = offset_y;


        _inputs[currentIndex] = data;


        return true;
    }

    void setMaximalDistance(int dist) 
    { 
        _maximal_distance_change = dist; 
    }

    bool processInput(double & newCost, InputData & input)
    {
        //Get bounding box of input in panoram
        //Dilate to have some pixels outside of the input
        BoundingBox localBbox = input.rect.dilate(3);
        localBbox.clampLeft();
        localBbox.clampTop();
        localBbox.clampBottom(_labels.getHeight() - 1);
        
        //Output must keep a margin also
        BoundingBox outputBbox = input.rect;
        outputBbox.left = input.rect.left - localBbox.left;
        outputBbox.top = input.rect.top - localBbox.top;

        //Extract labels for the ROI
        image::Image<IndexT> localLabels(localBbox.width, localBbox.height);
        if (!loopyCachedImageExtract(localLabels, _labels, localBbox)) 
        {
            return false;
        }   

        // Compute distance map to borders of the input seams
        image::Image<int> distanceMap(localLabels.Width(), localLabels.Height());
        {
            image::Image<IndexT> binarizedWorld(localLabels.Width(), localLabels.Height());

            for(int y = 0; y < localLabels.Height(); y++)
            {
                for(int x = 0; x < localLabels.Width(); x++)
                {
                    IndexT label = localLabels(y, x);

                    if(label == input.id)
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
        }


        //Build the input
        image::Image<PixelInfo> graphCutInput(localBbox.width, localBbox.height, true);
        
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

            BoundingBox intersection = localBbox.intersectionWith(otherBbox);
            BoundingBox intersectionLoop = localBbox.intersectionWith(otherBboxLoop);
            BoundingBox intersectionLoopRight = localBbox.intersectionWith(otherBboxLoopRight);
            if (intersection.isEmpty() && intersectionLoop.isEmpty() && intersectionLoopRight.isEmpty())
            {
                continue;
            }
            
            image::Image<image::RGBfColor> otherColor(otherInputBbox.width, otherInputBbox.height);
            if (!otherInput.second.color.extract(otherColor, otherInputBbox, otherInputBbox)) 
            {
                return false;
            }

            image::Image<unsigned char> otherMask(otherInputBbox.width, otherInputBbox.height);
            if (!otherInput.second.mask.extract(otherMask, otherInputBbox, otherInputBbox)) 
            {
                return false;
            }


            if (!intersection.isEmpty())
            {        
                BoundingBox interestOther = intersection;
                interestOther.left -= otherBbox.left;
                interestOther.top -= otherBbox.top;

                BoundingBox interestThis = intersection;
                interestThis.left -= localBbox.left;
                interestThis.top -= localBbox.top;

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
                        
                        float dist = sqrt(float(distanceMap(y_current, x_current)));
                        if (dist > _maximal_distance_change)
                        {
                            continue;
                        }

                        PixelInfo & pix = graphCutInput(y_current, x_current);
                        pix[otherInput.first] = otherColor(y_other, x_other);
                    }
                }
            }

            if (!intersectionLoop.isEmpty())
            {
                BoundingBox interestOther = intersectionLoop;
                interestOther.left -= otherBboxLoop.left;
                interestOther.top -= otherBboxLoop.top;

                BoundingBox interestThis = intersectionLoop;
                interestThis.left -= localBbox.left;
                interestThis.top -= localBbox.top;

                
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

                        float dist = sqrt(float(distanceMap(y_current, x_current)));
                        if (dist > _maximal_distance_change)
                        {
                            continue;
                        }

                        PixelInfo & pix = graphCutInput(y_current, x_current);
                        pix[otherInput.first] = otherColor(y_other, x_other);
                    }
                }
            }

            if (!intersectionLoopRight.isEmpty())
            {
                BoundingBox interestOther = intersectionLoopRight;
                interestOther.left -= otherBboxLoopRight.left;
                interestOther.top -= otherBboxLoopRight.top;

                BoundingBox interestThis = intersectionLoopRight;
                interestThis.left -= localBbox.left;
                interestThis.top -= localBbox.top;

                
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

                        float dist = sqrt(float(distanceMap(y_current, x_current)));
                        if (dist > _maximal_distance_change)
                        {
                            continue;
                        }

                        PixelInfo & pix = graphCutInput(y_current, x_current);
                        pix[otherInput.first] = otherColor(y_other, x_other);
                    }
                }
            }
        }

        //Because of upscaling, some labels may be incorrect
        //Some pixels may be affected to labels they don't see.
        for (int y = 0; y < graphCutInput.Height(); y++) 
        {
            for (int x = 0; x < graphCutInput.Width(); x++) 
            {
                IndexT label = localLabels(y, x);

                if (label == UndefinedIndexT)
                {
                    continue;
                }

                PixelInfo & pix = graphCutInput(y, x);

                if (pix.size() == 0)
                {
                    localLabels(y, x) = UndefinedIndexT;
                    continue;
                }

                auto it = pix.find(label);
                if (it == pix.end())
                {
                    localLabels(y, x) = pix.begin()->first;
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

            if (!loopyCachedImageAssign(_labels, localLabels, localBbox, inputBb)) 
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
            std::cout << "**************************" << std::endl;
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
                    std::cout << costs[info.first] << " ----> " << cost << std::endl;

                    costs[info.first] = cost;
                    hasChange = true;
                }
            }

            if (!hasChange)
            {
                break;
            }
        }

        char filename[512];
        sprintf(filename, "/home/mmoc/test%d.exr", _outputWidth);
        _labels.writeImage(filename);

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


                auto it = input(y, x).find(label);
                if (it != input(y, x).end())
                {
                    hasCLC = true;
                    CColorLC = it->second;
                }

                it = input(y, x).find(labelx);
                if (it != input(y, x).end())
                {
                    hasCLX = true;
                    CColorLX = it->second;
                }

                it = input(y, x).find(labely);
                if (it != input(y, x).end())
                {
                    hasCLY = true;
                    CColorLY = it->second;
                }

                it = input(y, xp).find(label);
                if (it != input(y, xp).end())
                {
                    hasXLC = true;
                    XColorLC = it->second;
                }

                it = input(y, xp).find(labelx);
                if (it != input(y, xp).end())
                {
                    hasXLX = true;
                    XColorLX = it->second;
                }

                it = input(yp, x).find(label);
                if (it != input(yp, x).end())
                {
                    hasYLC = true;
                    YColorLC = it->second;
                }

                it = input(yp, x).find(labely);
                if (it != input(yp, x).end())
                {
                    hasYLY = true;
                    YColorLY = it->second;
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

                auto it = input(y, x).find(currentLabel);
                if (it != input(y, x).end())
                {
                    currentColor = it->second;
                    mask(y, x) = 1;
                }

                if (label != currentLabel)
                {
                    it = input(y, x).find(label);
                    if (it != input(y, x).end())
                    {
                        otherColor = it->second;

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

    CachedImage<IndexT> & getLabels() 
    { 
        return _labels; 
    }

private:

    std::map<IndexT, InputData> _inputs;

    int _outputWidth;
    int _outputHeight;
    size_t _maximal_distance_change;
    CachedImage<IndexT> _labels;
};

} // namespace aliceVision