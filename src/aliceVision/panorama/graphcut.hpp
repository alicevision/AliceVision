#pragma once

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/boykov_kolmogorov_max_flow.hpp>

#include <aliceVision/image/all.hpp>

#include "distance.hpp"
#include "boundingBox.hpp"

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
    using PixelInfo = std::pair<IndexT, image::RGBfColor>;
    using ImageOwners = image::Image<std::vector<PixelInfo>>;

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
        
        if (!_original_labels.deepCopy(existing_labels)) 
        {
            return false;
        }

        /*image::Image<unsigned char> seams(_labels.Width(), _labels.Height());
        computeSeamsMap(seams, _labels);
        computeDistanceMap(_distancesSeams, seams);*/

        return true;
    }

    bool initialize(image::TileCacheManager::shared_ptr & cacheManager) 
    {
        if(!_labels.createImage(cacheManager, _outputWidth, _outputHeight))
        {
            return false;
        }

        if(!_original_labels.createImage(cacheManager, _outputWidth, _outputHeight))
        {
            return false;
        }

        /*if(!_distancesSeams.createImage(cacheManager, _outputWidth, _outputHeight))
        {
            return false;
        }

        if(!_distancesSeams.fill(0.0f)) 
        {
            return false;
        }    */


        return true;
    }

    bool append(const aliceVision::image::Image<image::RGBfColor>& input, const aliceVision::image::Image<unsigned char>& inputMask, IndexT currentIndex, size_t offset_x, size_t offset_y)
    {

       /* if(inputMask.size() != input.size())
        {
            return false;
        }

        BoundingBox rect;

        rect.left = offset_x;
        rect.top = offset_y;
        rect.width = input.Width() + 1;
        rect.height = input.Height() + 1;

        //Extend rect for borders
        rect.dilate(3);
        rect.clampLeft();
        rect.clampTop();
        rect.clampBottom(_owners.Height() - 1)

        _rects[currentIndex] = rect;

        
        //_owners will get for each pixel of the panorama a list of pixels
        //in the sources which may have seen this point.
        
        for(int i = 0; i < input.Height(); i++)
        {

            int di = i + offset_y;

            for(int j = 0; j < input.Width(); j++)
            {

                if(!inputMask(i, j))
                    continue;

                int dj = j + offset_x;
                if(dj >= _owners.Width())
                {
                    dj = dj - _owners.Width();
                }

                PixelInfo info;
                info.first = currentIndex;
                info.second = input(i, j);

                // If too far away from seam, do not add a contender
                int dist = _distancesSeams(di, dj);
                if(dist > _maximal_distance_change + 10)
                {
                    continue;
                }

                _owners(di, dj).push_back(info);
            }
        }*/

        return true;
    }

    void setMaximalDistance(int dist) 
    { 
        _maximal_distance_change = dist; 
    }

    bool process()
    {

        /*for(int i = 0; i < 10; i++)
        {

            // For each possible label, try to extends its domination on the label's world
            bool change = false;

            for(auto & info : _rects)
            {

                ALICEVISION_LOG_INFO("Graphcut expansion (iteration " << i << ") for label " << info.first);

                int p1 = info.second.left;
                int w1 = info.second.width;
                int p2 = 0;
                int w2 = 0;

                if(p1 + w1 > _labels.Width())
                {
                    w1 = _labels.Width() - p1;
                    p2 = 0;
                    w2 = info.second.width - w1;
                }

                Eigen::Matrix<IndexT, Eigen::Dynamic, Eigen::Dynamic> backup_1 = _labels.block(info.second.top, p1, info.second.height, w1);
                Eigen::Matrix<IndexT, Eigen::Dynamic, Eigen::Dynamic> backup_2 = _labels.block(info.second.top, p2, info.second.height, w2);

                double base_cost = cost(info.first);
                alphaExpansion(info.first);
                double new_cost = cost(info.first);

                if(new_cost > base_cost)
                {
                    _labels.block(info.second.top, p1, info.second.height, w1) = backup_1;
                    _labels.block(info.second.top, p2, info.second.height, w2) = backup_2;
                }
                else if(new_cost < base_cost)
                {
                    change = true;
                }
            }

            if(!change)
            {
                break;
            }
        }*/

        return true;
    }

    double cost(IndexT currentLabel)
    {

        BoundingBox rect = _rects[currentLabel];

        double cost = 0.0;

        /*for(int i = 0; i < rect.height - 1; i++)
        {

            int y = rect.top + i;
            int yp = y + 1;

            for(int j = 0; j < rect.width; j++)
            {

                int x = rect.left + j;
                if(x >= _owners.Width())
                {
                    x = x - _owners.Width();
                }

                int xp = x + 1;
                if(xp >= _owners.Width())
                {
                    xp = xp - _owners.Width();
                }

                IndexT label = _labels(y, x);
                IndexT labelx = _labels(y, xp);
                IndexT labely = _labels(yp, x);

                if(label == UndefinedIndexT)
                    continue;
                if(labelx == UndefinedIndexT)
                    continue;
                if(labely == UndefinedIndexT)
                    continue;

                if(label == labelx)
                {
                    continue;
                }

                image::RGBfColor CColorLC;
                image::RGBfColor CColorLX;
                image::RGBfColor CColorLY;
                bool hasCLC = false;
                bool hasCLX = false;
                bool hasCLY = false;

                for(int l = 0; l < _owners(y, x).size(); l++)
                {
                    if(_owners(y, x)[l].first == label)
                    {
                        hasCLC = true;
                        CColorLC = _owners(y, x)[l].second;
                    }

                    if(_owners(y, x)[l].first == labelx)
                    {
                        hasCLX = true;
                        CColorLX = _owners(y, x)[l].second;
                    }

                    if(_owners(y, x)[l].first == labely)
                    {
                        hasCLY = true;
                        CColorLY = _owners(y, x)[l].second;
                    }
                }

                image::RGBfColor XColorLC;
                image::RGBfColor XColorLX;
                bool hasXLC = false;
                bool hasXLX = false;

                for(int l = 0; l < _owners(y, xp).size(); l++)
                {
                    if(_owners(y, xp)[l].first == label)
                    {
                        hasXLC = true;
                        XColorLC = _owners(y, xp)[l].second;
                    }

                    if(_owners(y, xp)[l].first == labelx)
                    {
                        hasXLX = true;
                        XColorLX = _owners(y, xp)[l].second;
                    }
                }

                image::RGBfColor YColorLC;
                image::RGBfColor YColorLY;
                bool hasYLC = false;
                bool hasYLY = false;

                for(int l = 0; l < _owners(yp, x).size(); l++)
                {
                    if(_owners(yp, x)[l].first == label)
                    {
                        hasYLC = true;
                        YColorLC = _owners(yp, x)[l].second;
                    }

                    if(_owners(yp, x)[l].first == labely)
                    {
                        hasYLY = true;
                        YColorLY = _owners(yp, x)[l].second;
                    }
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

                cost += (CColorLC - CColorLX).norm();
                cost += (CColorLC - CColorLY).norm();
                cost += (XColorLC - XColorLX).norm();
                cost += (YColorLC - YColorLY).norm();
            }
        }*/

        return cost;
    }

    bool alphaExpansion(IndexT currentLabel)
    {
        /*
        BoundingBox rect = _rects[currentLabel];

        image::Image<unsigned char> mask(rect.width, rect.height, true, 0);
        image::Image<int> ids(rect.width, rect.height, true, -1);
        image::Image<image::RGBfColor> color_label(rect.width, rect.height, true, image::RGBfColor(0.0f, 0.0f, 0.0f));
        image::Image<image::RGBfColor> color_other(rect.width, rect.height, true, image::RGBfColor(0.0f, 0.0f, 0.0f));

        // Compute distance map to seams
        image::Image<int> distanceMap(rect.width, rect.height);
        {
            image::Image<IndexT> binarizedWorld(rect.width, rect.height);

            for(int i = 0; i < rect.height; i++)
            {
                int y = rect.top + i;

                for(int j = 0; j < rect.width; j++)
                {

                    int x = rect.left + j;
                    if(x >= _owners.Width())
                    {
                        x = x - _owners.Width();
                    }

                    IndexT label = _original_labels(y, x);
                    if(label == currentLabel)
                    {
                        binarizedWorld(i, j) = 1;
                    }
                    else
                    {
                        binarizedWorld(i, j) = 0;
                    }
                }
            }

            image::Image<unsigned char> seams(rect.width, rect.height);
            if(!computeSeamsMap(seams, binarizedWorld))
            {
                return false;
            }

            if(!computeDistanceMap(distanceMap, seams))
            {
                return false;
            }
        }

        
        //A warped input has valid pixels only in some parts of the final image.
        //Rect is the bounding box of these valid pixels.
        //Let's build a mask :
        // - 0 if the pixel is not viewed by anyone
        // - 1 if the pixel is viewed by the current label alpha
        // - 2 if the pixel is viewed by *another* label and this label is marked as current valid label
        // - 3 if the pixel is 1 + 2 : the pixel is not selected as alpha territory, but alpha is looking at it
        
        for(int i = 0; i < rect.height; i++)
        {

            int y = rect.top + i;

            for(int j = 0; j < rect.width; j++)
            {

                int x = rect.left + j;
                if(x >= _owners.Width())
                {
                    x = x - _owners.Width();
                }

                std::vector<PixelInfo>& infos = _owners(y, x);
                IndexT label = _labels(y, x);

                image::RGBfColor currentColor;
                image::RGBfColor otherColor;

                int dist = distanceMap(i, j);

                // Loop over observations
                for(int l = 0; l < infos.size(); l++)
                {

                    if(dist > _maximal_distance_change)
                    {

                        if(infos[l].first == label)
                        {
                            if(label == currentLabel)
                            {
                                mask(i, j) = 1;
                                currentColor = infos[l].second;
                            }
                            else
                            {
                                mask(i, j) = 2;
                                otherColor = infos[l].second;
                            }
                        }
                    }
                    else
                    {
                        if(infos[l].first == currentLabel)
                        {
                            mask(i, j) |= 1;
                            currentColor = infos[l].second;
                        }
                        else if(infos[l].first == label)
                        {
                            mask(i, j) |= 2;
                            otherColor = infos[l].second;
                        }
                    }
                }

                // If the pixel may be a new kingdom for alpha 
                if(mask(i, j) == 1)
                {
                    color_label(i, j) = currentColor;
                    color_other(i, j) = currentColor;
                }
                else if(mask(i, j) == 2)
                {
                    color_label(i, j) = otherColor;
                    color_other(i, j) = otherColor;
                }
                else if(mask(i, j) == 3)
                {
                    color_label(i, j) = currentColor;
                    color_other(i, j) = otherColor;
                }
            }
        }

        // The rectangle is a grid.
        // However we want to ignore a lot of pixel.
        // Let's create an index per valid pixels for graph cut reference
        int count = 0;
        for(int i = 0; i < rect.height; i++)
        {
            for(int j = 0; j < rect.width; j++)
            {
                if(mask(i, j) == 0)
                {
                    continue;
                }

                ids(i, j) = count;
                count++;
            }
        }

        //Create graph
        MaxFlow_AdjList gc(count);
        size_t countValid = 0;

        for(int i = 0; i < rect.height; i++)
        {
            for(int j = 0; j < rect.width; j++)
            {

                // If this pixel is not valid, ignore 
                if(mask(i, j) == 0)
                {
                    continue;
                }

                // Get this pixel ID 
                int node_id = ids(i, j);

                int im1 = std::max(i - 1, 0);
                int jm1 = std::max(j - 1, 0);
                int ip1 = std::min(i + 1, rect.height - 1);
                int jp1 = std::min(j + 1, rect.width - 1);

                if(mask(i, j) == 1)
                {

                    // Only add nodes close to borders 
                    if(mask(im1, jm1) == 1 && mask(im1, j) == 1 && mask(im1, jp1) == 1 && mask(i, jm1) == 1 &&
                       mask(i, jp1) == 1 && mask(ip1, jm1) == 1 && mask(ip1, j) == 1 && mask(ip1, jp1) == 1)
                    {
                        continue;
                    }

                    
                    //This pixel is only seen by alpha.
                    //Enforce its domination by stating that removing this pixel
                    //from alpha territoy is infinitly costly (impossible).
                    gc.addNodeToSource(node_id, 100000);
                }
                else if(mask(i, j) == 2)
                {
                    // Only add nodes close to borders
                    if(mask(im1, jm1) == 2 && mask(im1, j) == 2 && mask(im1, jp1) == 2 && mask(i, jm1) == 2 &&
                       mask(i, jp1) == 2 && mask(ip1, jm1) == 2 && mask(ip1, j) == 2 && mask(ip1, jp1) == 2)
                    {
                        continue;
                    }

                    //This pixel is only seen by an ennemy.
                    //Enforce its domination by stating that removing this pixel
                    //from ennemy territory is infinitly costly (impossible).
                    gc.addNodeToSink(node_id, 100000);
                }
                else if(mask(i, j) == 3)
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
        
        for(int i = 0; i < rect.height; i++)
        {
            for(int j = 0; j < rect.width; j++)
            {

                if(mask(i, j) == 0)
                {
                    continue;
                }

                int node_id = ids(i, j);

                // Make sure it is possible to estimate this horizontal border
                if(i < mask.Height() - 1)
                {

                    // Make sure the other pixel is owned by someone
                    if(mask(i + 1, j))
                    {

                        int other_node_id = ids(i + 1, j);
                        float w = 1000;

                        if(((mask(i, j) & 1) && (mask(i + 1, j) & 2)) || ((mask(i, j) & 2) && (mask(i + 1, j) & 1)))
                        {
                            float d1 = (color_label(i, j) - color_other(i, j)).norm();
                            float d2 = (color_label(i + 1, j) - color_other(i + 1, j)).norm();

                            d1 = std::min(2.0f, d1);
                            d2 = std::min(2.0f, d2);

                            w = (d1 + d2) * 100.0 + 1.0;
                        }

                        gc.addEdge(node_id, other_node_id, w, w);
                    }
                }

                if(j < mask.Width() - 1)
                {

                    if(mask(i, j + 1))
                    {

                        int other_node_id = ids(i, j + 1);
                        float w = 1000;

                        if(((mask(i, j) & 1) && (mask(i, j + 1) & 2)) || ((mask(i, j) & 2) && (mask(i, j + 1) & 1)))
                        {
                            float d1 = (color_label(i, j) - color_other(i, j)).norm();
                            float d2 = (color_label(i, j + 1) - color_other(i, j + 1)).norm();
                            w = (d1 + d2) * 100.0 + 1.0;
                        }

                        gc.addEdge(node_id, other_node_id, w, w);
                    }
                }
            }
        }

        gc.compute();

        int changeCount = 0;
        for(int i = 0; i < rect.height; i++)
        {

            int y = rect.top + i;

            for(int j = 0; j < rect.width; j++)
            {

                int x = rect.left + j;
                if(x >= _owners.Width())
                {
                    x = x - _owners.Width();
                }

                IndexT label = _labels(y, x);
                int id = ids(i, j);

                if(gc.isSource(id))
                {

                    if(label != currentLabel)
                    {
                        changeCount++;
                    }

                    _labels(y, x) = currentLabel;
                }
            }
        }*/

        return true;
    }

    CachedImage<IndexT> & getLabels() 
    { 
        return _labels; 
    }

private:

    std::map<IndexT, BoundingBox> _rects;
    int _outputWidth;
    int _outputHeight;
    size_t _maximal_distance_change;

    CachedImage<IndexT> _labels;
    CachedImage<IndexT> _original_labels;
    CachedImage<int> _distancesSeams;
    ImageOwners _owners;
};

} // namespace aliceVision