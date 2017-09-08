#include "mv_plyloader.h"
#include "stdafx.h"

struct refine_tri
{
    int* i;
    int size, n;
    bool alive;

    refine_tri()
    {
        i = nullptr;
        size = 0;
        n = 0;
        alive = true;
    }
};

static int vertex_cb(p_ply_argument argument)
{
    point3d* pts;
    long uservalue;
    p_ply_element element;
    long index;

    // If we need access to the element
    ply_get_argument_element(argument, &element, &index);

    // The user pointer tells us where the mesh is
    ply_get_argument_user_data(argument, ((void**)(&pts)), &uservalue);

    double value = ply_get_argument_value(argument);

    if(uservalue == 0)
    {
        pts[index].x = value;
    }
    if(uservalue == 1)
    {
        pts[index].y = value;
    }
    if(uservalue == 2)
    {
        pts[index].z = value;
    }

    return 1;
}

static int face_cb(p_ply_argument argument)
{
    refine_tri* ids;
    long uservalue;
    p_ply_element element;
    long index;

    ply_get_argument_element(argument, &element, &index);

    ply_get_argument_user_data(argument, ((void**)(&ids)), &uservalue);

    double value = ply_get_argument_value(argument);

    if(ids[index].size > 0)
    {
        ids[index].i[ids[index].n] = (int)value;
        ids[index].n++;
    }

    if(ids[index].size == 0)
    {
        ids[index].size = (int)value;
        ids[index].i = new int[ids[index].size];
    }

    return 1;
}

bool mv_loadply(std::string plyFileName, mv_mesh* me)
{
    printf("Reading file %s \n", plyFileName.c_str());

    // Open the ply file
    p_ply ply = ply_open(plyFileName.c_str(), nullptr);
    if(ply == nullptr)
    {
        printf("Could not open ply file %s\n", plyFileName.c_str());
        return false;
    }

    if(ply_read_header(ply) == 0)
    {
        printf("Error reading header for ply file %s\n", plyFileName.c_str());
        return false;
    }

    int nvertices = ply_set_read_cb(ply, "vertex", "x", vertex_cb, nullptr, 0);
    int ntriangles = ply_set_read_cb(ply, "face", "vertex_indices", face_cb, nullptr, 0);

    point3d* pts = new point3d[nvertices];
    void* ptrpts = (void*)pts;

    refine_tri* ids = new refine_tri[ntriangles];
    for(int i = 0; i < ntriangles; i++)
    {
        ids[i] = refine_tri();
    }
    void* ptrids = (void*)ids;

    ply_set_read_cb(ply, "vertex", "x", vertex_cb, ptrpts, 0);
    ply_set_read_cb(ply, "vertex", "y", vertex_cb, ptrpts, 1);
    ply_set_read_cb(ply, "vertex", "z", vertex_cb, ptrpts, 2);

    ply_set_read_cb(ply, "face", "vertex_indices", face_cb, ptrids, 0);

    // Read the file
    if(ply_read(ply) == 0)
    {
        ply_close(ply);
        printf("Error reading ply file %s\n", plyFileName.c_str());
        return false;
    }

    // Close the file handle
    ply_close(ply);

    int ntris = 0;
    for(int i = 0; i < ntriangles; i++)
    {
        if(ids[i].size == 3)
        {
            ntris++;
        }
    }

    printf("readed %i vertices, %i triangels of %i faces \n", nvertices, ntris, ntriangles);

    if(ntris != ntriangles)
    {
        printf("warning ntriangles != nfaces ... may cause a failure !!!! \n");
    }

    ////////////////////////////////////////////////////////////////////////
    // fill mesh

    // me->pts = new staticVector<scenePoint>(nvertices);
    me->pts = new staticVector<point3d>(nvertices);
    for(int i = 0; i < nvertices; i++)
    {
        /*
        scenePoint m;
        m.op.p = pts[i];
        m.op.n = point3d();
        m.op.sim = 1.0;
        m.refImgFileId = -1;
        m.seedId = -1;
        m.uniqueId = -1;
        me->pts->push_back(m);
        */
        me->pts->push_back(pts[i]);
    }

    me->tris = new staticVector<mv_mesh::triangle>(ntriangles);
    for(int i = 0; i < ntriangles; i++)
    {
        mv_mesh::triangle t;
        t.i[0] = ids[i].i[0];
        t.i[1] = ids[i].i[1];
        t.i[2] = ids[i].i[2];
        t.alive = ids[i].alive;
        me->tris->push_back(t);
    }

    for(int i = 0; i < ntriangles; i++)
    {
        if(ids[i].size > 0)
        {
            delete[] ids[i].i;
        }
    }
    delete[] ids;
    delete[] pts;

    return true;
}

/*
int mv_loadply(std::string plyFileName, mv_mesh *me)
{
        printf("Reading from file %s \n",plyFileName.c_str());

        FILE* plyf = 0;


        bool dobinary = false;


    if (dobinary==true)
        {
                plyf = fopen(plyFileName.c_str(),"rb");
        }else
        {
                plyf = fopen(plyFileName.c_str(),"r");
        };


        if (plyf == 0) {
          _cprintf("Could not open output ply file %s\n",plyFileName.c_str());
          return -1;
        }


        freadf(plyf,"ply%c",10);
    if (dobinary==true)
        {
                fprintf(plyf,"format binary_little_endian 1.0%c",10);
        }else
        {
                fprintf(plyf,"format ascii 1.0%c",10);
        };
    fprintf(plyf,"comment Generated by multiRecon ( http://cmp.felk.cvut.cz/~jancom1 )%c",10);
    fprintf(plyf,"element vertex %d%c",me->pts->size(),10);
    fprintf(plyf,"property float x%c",10);
    fprintf(plyf,"property float y%c",10);
    fprintf(plyf,"property float z%c",10);
    fprintf(plyf,"property uchar diffuse_red%c",10);
    fprintf(plyf,"property uchar diffuse_green%c",10);
    fprintf(plyf,"property uchar diffuse_blue%c",10);
    fprintf(plyf,"element face %d%c",me->tris->size(),10);
    fprintf(plyf,"property list uchar int vertex_indices%c",10);
    fprintf(plyf,"end_header%c",10);


        for (int i=0;i<me->pts->size();i++)
        {
                if (dobinary==true)
                {
                        float pos[3];
                        pos[0] = (float)((*me->pts)[i].x);
                        pos[1] = (float)((*me->pts)[i].y);
                        pos[2] = (float)((*me->pts)[i].z);
                        fwrite(&pos[0],sizeof(float),3,plyf);
                }else
                {
                        fprintf(plyf,"%f %f %f 128 128 128%c",
                                (float)((*me->pts)[i].x),
                                (float)((*me->pts)[i].y),
                                (float)((*me->pts)[i].z),
                                10
                        );
                };
        };

        for (int i=0;i<me->tris->size();i++)
        {
                if (dobinary==true)
                {
                        unsigned char npts = 3;
                        fwrite(&npts,sizeof(unsigned char),1,plyf);
                        int points[3];
                        points[0] = (*me->tris)[i].i[0];
                        points[1] = (*me->tris)[i].i[1];
                        points[2] = (*me->tris)[i].i[2];
                        fwrite(&points[0],sizeof(int),3,plyf);
                }else
                {
                        fprintf(plyf,"3 %d %d %d",
                                (*me->tris)[i].i[0],
                                (*me->tris)[i].i[1],
                                (*me->tris)[i].i[2]
                        );

                        if (i < me->tris->size() - 1)
                        {
                                fprintf(plyf,"%c",10);
                        };

                };
        }

        fclose(plyf);

    return 0;
}
*/