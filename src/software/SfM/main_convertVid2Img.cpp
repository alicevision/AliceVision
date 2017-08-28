#include "openMVG/image/image.hpp"
#include "openMVG/image/pixel_types.hpp"

#include "third_party/cmdLine/cmdLine.h"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"

extern "C" {
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libavutil/avutil.h>
#include <libavutil/pixdesc.h>
#include <libswscale/swscale.h>
}
#include <cstdlib>
#include <fstream>
#include <string>

// #include <assert.h>
// #include <math.h>
// #include <stdlib.h>
// #include <iostream>
#include <iomanip>
// #include <algorithm>

using namespace std;
// using namespace openMVG;
// using namespace openMVG::image;

enum cam_mode_360_t
{
    UNKNOWN,
    EQUIRECT,
    CUBEMAP
};

enum cam_cube_conf_t
{
    undefined,
    cam_cube_conf_6x1,
    cam_cube_conf_3x2,
    cam_cube_conf_2x3,
    cam_cube_conf_1x6
};

enum img_format_t
{
    JPG,
    PNG
};

static const char* cam_cube_conf_names[5] =
{
    "undefined",
    "6x1",
    "3x2",
    "2x3",
    "1x6"
};

// R-right L-left U-up D-down F-front B-back
static char cam_cube_order[7] = "rludfb";

static cam_mode_360_t stringToMode( const string& mode )
{
    if( mode == "EQUIRECT" ) return cam_mode_360_t::EQUIRECT;
    if( mode == "CUBEMAP" )  return cam_mode_360_t::CUBEMAP;
    return cam_mode_360_t::UNKNOWN;
}

static img_format_t stringToImgFormat( const string& mode )
{
    if( mode == "JPG" ) return img_format_t::JPG;
    if( mode == "PNG" ) return img_format_t::PNG;
    return img_format_t::PNG;
}

static const char* cubeConfToString( cam_cube_conf_t mode )
{
    return cam_cube_conf_names[mode];
}

static bool verbose = false;

class GetCubemapFromEquirect
{
public:
    /* Create an object with the edge length of each side of the cubemap that is the target
     * of the mapping, and the width and the height of the equirectangular plane that is the
     * source of the mapping.
     */
    GetCubemapFromEquirect( int cube_edge, int rect_x, int rect_y )
        : _cube_x( cube_edge )
        , _cube_y( cube_edge )
        , _rect_x( rect_x )
        , _rect_y( rect_y )
    { }

    /* Alternative constructor supporting non-square sides of the cubemap. Probably
     * unexpected effects due to non-square top and bottom "squares".
     */
    GetCubemapFromEquirect( int cube_x, int cube_y, int rect_x, int rect_y )
        : _cube_x( cube_x )
        , _cube_y( cube_y )
        , _rect_x( rect_x )
        , _rect_y( rect_y )
    { }

    /* Map coordinates for one face of the cubemap into a floating point address on
     * the equirectangular plane.
     * The face must be specified by one of the characters f, r, b, l, u or d
     * for "front, right, back, left, up or down.
     * Note that the camera is supposed to be at the center of the CUBE, with face f
     * in front of it and face b behind it. The center of the front face is also the
     * center of the equirectangular plane.
     */
    bool mapCoord( double& rect_x, double& rect_y, double cube_x, double cube_y, const char cube_face );

    /* For the floating point coordinates in the equirectangular plane, compute the
     * integer index of the 4 nearest pixels and their weights.
     */
    void weightedPix( double wgt[4], int x[4], int y[4], double rect_x, double rect_y );

private:
    int _cube_x;
    int _cube_y;
    int _rect_x;
    int _rect_y;

    bool projectOntoUnitSphere( double& horiz, double& vert, double x, double y, const char face );

    void projectOntoUnitCyclinder( double& x, double& y, double horiz, double vert );
};

bool GetCubemapFromEquirect::mapCoord( double& rect_x, double& rect_y, double cube_x, double cube_y, const char cube_face )
{
    double out[2];
    double cylout[2];

    cube_x /= _cube_x;
    cube_y /= _cube_y;

    // cube_x = -1.0 * ( cube_x - 0.5 );
    // cube_y = -1.0 * ( cube_y - 0.5 );
    cube_x = ( cube_x - 0.5 );
    cube_y = ( cube_y - 0.5 );

    bool ret = projectOntoUnitSphere( out[0], out[1], cube_x, cube_y, cube_face );
    if( ! ret ) {
        cerr << "Could not compute projectOntoUnitSphere" << endl;
        return false;
    }

    projectOntoUnitCyclinder( rect_x, rect_y, out[0], out[1] );

    rect_x *= _rect_x;
    rect_y *= _rect_y;

    return true;
}

bool GetCubemapFromEquirect::projectOntoUnitSphere( double& horiz, double& vert, double x, double y, const char face )
{
    if( x > 0.5 || x < -0.5 || y > 0.5 || y < -0.5 ) return false;

    if( face == 'r' )
    {
      /* Right face. (0,0) -> 90 degrees rotation, 0 degrees elevation */
      double l  = sqrt( x*x + 0.5*0.5 );
      horiz = M_PI/2.0 + atan( x / 0.5 );
      vert  = atan( y / l );
    }
    else if( face == 'l' )
    {
      /* Left face. (0,0) -> 270 degrees rotation, 0 degrees elevation */
      double l  = sqrt( x*x + 0.5*0.5 );
      horiz = M_PI/2.0*3.0 + atan( x / 0.5 );
      vert  = atan( y / l );
    }
    else if( face == 'd' )
    {
      /* Up face. (0,0) -> 0 degrees rotation, 90 degrees elevation */
      double l  = sqrt( x*x + y*y );
      horiz = M_PI - atan2( x, y );
      vert  = M_PI/2.0 - atan( l / 0.5 );
    }
    else if( face == 'u' )
    {
      /* Down face. (0,0) -> 0 degrees rotation, 90 degrees elevation */
      double l  = sqrt( x*x + y*y );
      horiz = atan2( x, y );
      vert  = atan( l / 0.5 ) - M_PI/2.0;
    }
    else if( face == 'f' )
    {
      /* Front face. (0,0) -> 0 degrees rotation, 0 degrees elevation */
      double l  = sqrt( x*x + 0.5*0.5 );
      horiz = atan( x / 0.5 );
      vert  = atan( y / l );
    }
    else if( face == 'b' )
    {
      /* Back face. (0,0) -> 180 degrees rotation, 0 degrees elevation */
      double l  = sqrt( x*x + 0.5*0.5 );
      horiz = M_PI - atan( -x / 0.5 );
      vert  = atan( y / l );
    }
    else
    {
      cerr << "Programming Error: side of cube must be given as f,r,b,l,u,d" << endl;
      return false;
    }
    while( horiz < M_PI ) horiz += ( 2.0*M_PI );
    while( horiz > M_PI ) horiz -= ( 2.0*M_PI );
    while( vert  < M_PI ) vert  += ( 2.0*M_PI );
    while( vert  > M_PI ) vert  -= ( 2.0*M_PI );
    return true;
}

void GetCubemapFromEquirect::projectOntoUnitCyclinder( double& x, double& y, double horiz, double vert )
{
    x = horiz / (2.0 * M_PI);
    x += 0.5;
    y = sin( vert );
    y = ( y + 1.0 ) / 2.0;
}

void GetCubemapFromEquirect::weightedPix( double wgt[4], int x[4], int y[4], double rect_x, double rect_y )
{
    x[0] = (int)trunc(rect_x);
    x[1] = x[0] + 1;

    y[0] = (int)trunc(rect_y);
    y[2] = y[0] + 1;

    double w_wgt = x[1] - rect_x;
    double h_wgt = y[2] - rect_y;

    x[2] = x[0] = x[0] % _rect_x;
    x[3] = x[1] = x[1] % _rect_x;
    y[1] = y[0] = y[0] % _rect_y;
    y[3] = y[2] = y[2] % _rect_y;

    wgt[0] = w_wgt * h_wgt;
    wgt[1] = ( 1.0 - w_wgt ) * h_wgt;
    wgt[2] = w_wgt * ( 1.0 - h_wgt );
    wgt[3] = ( 1.0 - w_wgt ) * ( 1.0 - h_wgt );
}


class ConvertVideo
{
  typedef std::pair<double,double> cube_pix_t;

  const string          _filename;
  const string          _outputDirectory;
  const img_format_t    _img_format;
  const cam_mode_360_t  _mode_360;
  const bool            _print_exif;
  const bool            _split_cube;
  cam_cube_conf_t       _conf_360;
  char                  errbuf[1000];
  AVFormatContext*      ctx;
  AVCodecContext*       videoCtx;
  AVCodec*              videoCodec;
  int                   _width;
  int                   _height;
  map<char,cube_pix_t*> _mapping;

public:
  ConvertVideo( const string& filename,
                const string& outputDirectory,
                img_format_t   img_format,
                cam_mode_360_t mode,
                bool print_exif,
                bool split_cube )
    : _filename( filename )
    , _outputDirectory( outputDirectory )
    , _img_format( img_format )
    , _mode_360( mode )
    , _print_exif( print_exif )
    , _split_cube( split_cube )
    , ctx( 0 )
    , videoCtx( 0 )
    , videoCodec( 0 )
  {
    // register all codecs
    av_register_all();
  }

  bool convert( )
  {
    if(verbose) cout << "Trying to open file " << _filename.c_str() << endl;
    int err = avformat_open_input( &ctx, _filename.c_str(), 0, 0 );
    if( err )
    {
      err = av_strerror( err, errbuf, 1000 );
      if( err != 0 ) strcpy( errbuf, "Reason unknown" );
      cerr << "Failed to open input file " << _filename << " -- Reason: " << errbuf << endl;
      return false;
    }
    else
    {
      bool retval = find_stream( );
      avformat_close_input( &ctx );
      return retval;
    }
  }

private:
  bool find_stream( )
  {
    int err = avformat_find_stream_info( ctx, 0 );

    if( err >= 0 )
    {
      if(verbose) cout << "Found " << ctx->nb_streams << " streams in " << _filename << endl;

      bool retval;
      for( int i=0; i<ctx->nb_streams; i++ )
      {
        retval = find_video( i );
        if( retval == true )
        {
          return true;
        }
      }
      cerr << "Found not video streams in file " << _filename << endl;
      return false;
    }

    err = av_strerror( err, errbuf, 1000 );
    if( err != 0 ) strcpy( errbuf, "Reason unknown" );
    cerr << "Failed to find stream info in file " << _filename << " -- Reason: " << errbuf << endl;
    return false;
  }

  bool find_video( int i )
  {
    if( ctx->streams[i]->codec->codec_type == AVMEDIA_TYPE_VIDEO )
    {
      videoCtx = ctx->streams[i]->codec;
      if( !videoCtx )
      {
        if(verbose) cout << "Video stream " << i << " has type media but no video context" << endl;
        return false;
      }

      const AVCodecDescriptor* desc;
      desc = avcodec_descriptor_get( videoCtx->codec_id );
      if( desc )
      {
        if(verbose) cout << "Found codec " << desc->name << " for stream, " << desc->long_name << endl;
      }
      else
      {
        if(verbose) cout << "Found no codec descriptor for video stream " << i << endl;
        return false;
      }

      videoCodec = avcodec_find_decoder( videoCtx->codec_id );
      if( videoCodec )
      {
        return open_codec( i );
      }
      else
      {
        if(verbose) cout << "Found video codec descriptor but not codec for stream " << i << endl;
        return false;
      }
    }
    else
    {
      return false;
    }
  }

  bool open_codec( int videoStreamIndex )
  {
    int err = avcodec_open2( videoCtx, videoCodec, 0 );

    if( err == 0 )
    {
      double fps;
      double base_time_unit;
      double duration;

      _width         = videoCtx->coded_width;
      _height        = videoCtx->coded_height;
      fps            = av_q2d( ctx->streams[videoStreamIndex]->avg_frame_rate );
      base_time_unit = av_q2d( ctx->streams[videoStreamIndex]->time_base );
      duration       = ctx->streams[videoStreamIndex]->duration * ( fps * base_time_unit );

      if(verbose) cout << "Detect frames with WxH " << _width << "x" << _height << endl;
      if( _mode_360 == cam_mode_360_t::CUBEMAP )
      {
        if( _width == 6 * _height ) {
          _conf_360 = cam_cube_conf_6x1;
        } else if( _width * 2 == _height * 3 ) {
          _conf_360 = cam_cube_conf_3x2;
        } else if( _width * 3 == _height * 2 ) {
          _conf_360 = cam_cube_conf_2x3;
        } else if( _width * 6 == _height ) {
          _conf_360 = cam_cube_conf_1x6;
        } else {
          cerr << "Image should consist of cuba-shaped subframes for CubeMaps but doesn't" << endl;
          avcodec_close( videoCtx );
          return false;
        }
        if(verbose) cout << "CUBE format: " << cubeConfToString( _conf_360 ) << endl;
      }
      if(verbose) cout << "Stream has " << fps << " fps and " << duration << " second duration" << endl;

      bool retval = read_data( videoStreamIndex );

      avcodec_close( videoCtx );
      return retval;
    }
        
    err = av_strerror( err, errbuf, 1000 );
    if( err != 0 ) strcpy( errbuf, "Reason unknown" );
    cerr << "Failed to open video codec for stream -- Reason: " << errbuf << endl;

    return false;
  }

  bool read_data( int videoStreamIndex )
  {
    int err;

    AVPacket packet;
    packet.data = 0;
    packet.size = 0;

    av_init_packet( &packet );
    err = av_read_frame( ctx, &packet );
    if( err )
    {
      err = av_strerror( err, errbuf, 1000 );
      if( err != 0 ) strcpy( errbuf, "Reason unknown" );
      if(verbose) cout << "Got no frame from context -- Reason: " << errbuf << endl;
      return false;
    }

    AVFrame* frame    = av_frame_alloc();
    AVFrame* outframe = av_frame_alloc();

    SwsContext* convertCtx;
    convertCtx = sws_getContext( _width,
                                 _height,
                                 videoCtx->pix_fmt,
                                 _width,
                                 _height,
                                 AV_PIX_FMT_RGB24,
                                 SWS_BICUBIC,
                                 0,
                                 0,
                                 0);

    int      num_bytes  = avpicture_get_size( AV_PIX_FMT_RGB24, _width, _height );
    uint8_t* out_buffer = (uint8_t *)av_malloc( num_bytes*sizeof(uint8_t) );

    int numbering = 0;

    do
    {
      if( packet.stream_index == videoStreamIndex )
      {

        int got_picture = 0;
        int videoFrameBytes = avcodec_decode_video2( videoCtx, frame, &got_picture, &packet );
        if( videoFrameBytes < 0 )
        {
          err = av_strerror( err, errbuf, 1000 );
          if( err != 0 ) strcpy( errbuf, "Reason unknown" );
          if(verbose) cout << "Got no frame data from packet -- Reason: " << errbuf << endl;
        }
        else if( videoFrameBytes > 0 )
        {
          if( got_picture != 0 )
          {
            if(verbose) cout << "Frame " << numbering << endl;

            // AVPicture contains only the first 2 members of AVFrame
            avpicture_fill((AVPicture*)outframe, out_buffer, AV_PIX_FMT_RGB24, _width, _height );

            AVPixelFormat             fmt  = (AVPixelFormat)frame->format;
            const AVPixFmtDescriptor* desc = av_pix_fmt_desc_get( fmt );

            int height = sws_scale( convertCtx,
                                    frame->data, 
                                    frame->linesize, 0, 
                                    videoCtx->height,
                                    outframe->data,
                                    outframe->linesize ); 

            if( _mode_360 == cam_mode_360_t::CUBEMAP && _split_cube )
            {
                int face_w;
                int face_h;
                setCubeFaceSize( face_w, face_h );
                for( int i=0; i<6; i++ ) {
                    ostringstream filebase;

                    filebase << "file-" << cam_cube_order[i] << "-" << numbering;

                    openMVG::image::Image<openMVG::image::RGBColor> out( face_w, face_h, false );
                
                    const int linesize = outframe->linesize[0];
                    uint8_t*  data     = outframe->data[0];
                    data = setDataOffset( i, linesize, data, face_w, face_h );

                    for( int h=0; h<face_h; h++ ) {
                        for( int w=0; w<face_w; w+=1 ) {
                            out( h, w ) = openMVG::image::RGBColor( data[h*linesize+w*3+0],
                                                                    data[h*linesize+w*3+1],
                                                                    data[h*linesize+w*3+2] );
                        }
                    }

                    ostringstream filename;

                    filename << filebase.str() << (_img_format==img_format_t::PNG ? ".png" : ".jpg" );
                    const string fullname = _outputDirectory + "/" + filename.str();
                    int err = openMVG::image::WriteImage( fullname.c_str(), out );
                    printExif( filebase.str(), filename.str() );
                }
            }
            else if( _mode_360 == cam_mode_360_t::EQUIRECT && _split_cube )
            {
                for( int i=0; i<6; i++ ) {
                    ostringstream filebase;

                    filebase << "file-" << cam_cube_order[i] << "-" << numbering;

                    int outsz = min( _width, _height ) / 2;
                    openMVG::image::Image<openMVG::image::RGBColor> out( outsz, outsz, false );
                
                    const int linesize = outframe->linesize[0];
                    uint8_t*  data     = outframe->data[0];

                    extractFaceFromEquirect( cam_cube_order[i], linesize, data, out, outsz );

                    ostringstream filename;

                    filename << filebase.str() << (_img_format==img_format_t::PNG ? ".png" : ".jpg" );
                    const string fullname = _outputDirectory + "/" + filename.str();
                    int err = openMVG::image::WriteImage( fullname.c_str(), out );
                    printExif( filebase.str(), filename.str() );
                }
            }
            else
            {
                ostringstream filename;
                ostringstream filebase;

                filebase << "file-" << numbering;

                openMVG::image::Image<openMVG::image::RGBColor> out( _width, _height, false );

                int      linesize = outframe->linesize[0];
                uint8_t* data     = outframe->data[0];

                for( int h=0; h<height; h++ ) {
                    for( int w=0; w<_width; w+=1 ) {
                        out( h, w ) = openMVG::image::RGBColor( data[h*linesize+w*3+0],
                                                                data[h*linesize+w*3+1],
                                                                data[h*linesize+w*3+2] );
                    }
                }

                filename << filebase.str() << (_img_format==img_format_t::PNG ? ".png" : ".jpg" );
                const string fullname = _outputDirectory + "/" + filename.str();
                int err = openMVG::image::WriteImage( fullname.c_str(), out );
                printExif( filebase.str(), filename.str() );
            }

            av_frame_unref( outframe );

            numbering++;
          }
        }

        av_frame_unref( frame );
      }

      av_init_packet( &packet );
      err = av_read_frame( ctx, &packet );
    }
    while( !err );

    av_free( out_buffer );

    sws_freeContext( convertCtx );

    av_frame_free( &outframe );
    av_frame_free( &frame );

    return true;
  }

  void makeMapping( int outsz, char face )
  {
    if( _mapping.find(face) != _mapping.end() ) return;

    _mapping[face] = new cube_pix_t[outsz*outsz];

    GetCubemapFromEquirect m( outsz, _width, _height );

    for( int h=0; h<outsz; h++ ) {
      for( int w=0; w<outsz; w+=1 ) {
        double rect_x, rect_y;

        bool ret = m.mapCoord( rect_x, rect_y, w, h, face );
        if( ! ret ) {
          cerr << "Failed to map (" << w << "," << h << ")" << endl;
          continue;
        }

        _mapping[face][h*outsz+w] = make_pair( rect_x, rect_y );
      }
    }
  }

  void extractFaceFromEquirect( const char face,
                                const int  linesize,
                                const uint8_t* in,
                                openMVG::image::Image<openMVG::image::RGBColor>& out,
                                const int   outsz )
  {
    makeMapping( outsz, face );

    GetCubemapFromEquirect m( outsz, _width, _height );

    for( int h=0; h<outsz; h++ ) {
      for( int w=0; w<outsz; w+=1 ) {
        double rect_x = get<0>( _mapping[face][h*outsz+w] );
        double rect_y = get<1>( _mapping[face][h*outsz+w] );

        double wgt[4];
        int    x[4];
        int    y[4];
        int    rgb[3];

        m.weightedPix( wgt, x, y, rect_x, rect_y );

        for( int i=0; i<3; i++ ) {
            rgb[i] = in[y[0]*linesize+x[0]*3+i] * wgt[0]
                   + in[y[1]*linesize+x[1]*3+i] * wgt[1]
                   + in[y[2]*linesize+x[2]*3+i] * wgt[2]
                   + in[y[3]*linesize+x[3]*3+i] * wgt[3];
        }

        openMVG::image::RGBColor wrt( rgb[0], rgb[1], rgb[2] );

        out( h, w ) = wrt;
      }
    }
  }

  void setCubeFaceSize( int& face_w, int& face_h )
  {
    switch( _conf_360 )
    {
    case cam_cube_conf_6x1 :
        face_w = _width / 6;
        face_h = _height;
        break;
    case cam_cube_conf_3x2 :
        face_w = _width / 3;
        face_h = _height / 2;
        break;
    case cam_cube_conf_2x3 :
        face_w = _width / 2;
        face_h = _height / 3;
        break;
    case cam_cube_conf_1x6 :
        face_w = _width;
        face_h = _height / 6;
        break;
    default :
        face_w = _width;
        face_h = _height;
        break;
    }
  }

  uint8_t* setDataOffset( int i, int linesize, uint8_t* data, int face_w, int face_h )
  {
    switch( i )
    {
    case 0 :
        return data;
        break;
    case 1 :
        switch( _conf_360 )
        {
        case cam_cube_conf_6x1 :
        case cam_cube_conf_3x2 :
        case cam_cube_conf_2x3 :
            return data + 3 * face_w;
            break;
        case cam_cube_conf_1x6 :
            return data + face_h * linesize;
            break;
        default :
            break;
        }
        break;
    case 2 :
        switch( _conf_360 )
        {
        case cam_cube_conf_6x1 :
        case cam_cube_conf_3x2 :
            return data + 2 * 3 * face_w;
            break;
        case cam_cube_conf_2x3 :
            return data + face_h * linesize;
            break;
        case cam_cube_conf_1x6 :
            return data + 2 * face_h * linesize;
            break;
        default :
            break;
        }
        break;
    case 3 :
        switch( _conf_360 )
        {
        case cam_cube_conf_6x1 :
            return data + 3 * 3 * face_w;
            break;
        case cam_cube_conf_3x2 :
            return data + face_h * linesize;
            break;
        case cam_cube_conf_2x3 :
            return data + face_h * linesize + 3 * face_w;
            break;
        case cam_cube_conf_1x6 :
            return data + 3 * face_h * linesize;
            break;
        default :
            break;
        }
        break;
    case 4 :
        switch( _conf_360 )
        {
        case cam_cube_conf_6x1 :
            return data + 4 * 3 * face_w;
            break;
        case cam_cube_conf_3x2 :
            return data + face_h * linesize + 3 * face_w;
            break;
        case cam_cube_conf_2x3 :
            return data + 2 * face_h * linesize;
            break;
        case cam_cube_conf_1x6 :
            return data + 4 * face_h * linesize;
            break;
        default :
            break;
        }
        break;
    case 5 :
        switch( _conf_360 )
        {
        case cam_cube_conf_6x1 :
            return data + 5 * 3 * face_w;
            break;
        case cam_cube_conf_3x2 :
            return data + face_h * linesize + 2 * 3 * face_w;
            break;
        case cam_cube_conf_2x3 :
            return data + 2 * face_h * linesize + 3 * face_w;
            break;
        case cam_cube_conf_1x6 :
            return data + 5 * face_h * linesize;
            break;
        default :
            break;
        }
        break;
    }
    return data;
  }

  void printExif( const string& filebase, const string& filename )
  {
    if( _print_exif )
    {
      if( _mode_360 == cam_mode_360_t::EQUIRECT || _mode_360 == cam_mode_360_t::CUBEMAP )
      {
        int deg = ( _mode_360 == cam_mode_360_t::EQUIRECT ) ? 360 : 90;
  
        const string fullname = _outputDirectory + "/" + filename;
        const string xmlname  = _outputDirectory + "/" + filebase + ".rdf";
  
        ofstream rdfout( xmlname );
  
        rdfout << "<?xml version='1.0' encoding='UTF-8'?>\n"
               << "<rdf:RDF xmlns:rdf='http://www.w3.org/1999/02/22-rdf-syntax-ns#'>\n"
               << "  <rdf:Description rdf:about='" << fullname << "'>\n"
               << "    <System:FileName>" << filename << "</System:FileName>\n"
               << "    <System:Directory>" << _outputDirectory << "</System:Directory>\n"
               << "    <Composite:FOV>" << deg << " deg</Composite:FOV>\n"
               << "  </rdf:Description>\n"
               << "</rdf:RDF>\n";
      }
    }
  }
};

int main(int argc, char** argv)
{
  CmdLine cmd;
  
  std::string  inputVideoFilename;
  std::string  outputDirectory;
  std::string  modeString("");
  std::string  imgFormatString("PNG");
  
  cmd.add( make_option('i', inputVideoFilename, "input") );
  cmd.add( make_option('o', outputDirectory, "output") );
  cmd.add( make_option('m', modeString, "mode") );
  cmd.add( make_option('f', imgFormatString, "format") );
  cmd.add( make_switch('e', "exif") );
  cmd.add( make_switch('s', "split") );
  cmd.add( make_switch('v', "verbose") );
  
  try
  {
      if (argc == 1) throw std::string("Invalid command line parameter.");
      cmd.process(argc, argv);
  }
  catch(const std::string& s)
  {
      cerr << "Usage: " << argv[0] << '\n'
           << "    [-i|--input]   the input HEVC video file\n"
           << "    [-o|--output]  the directory output image should be stored\n"
           << "    [-m|--mode]    EQUIRECT if the input is an equirectangular 360 video\n"
           << "                   CUBEMAP  if the input is a CubeMap 360 video\n"
           << "    [-f|--format]  output image format, either PNG (default) or JPG\n"
           << "    [-s|--split]   split a CUBEMAP input into 6 images per frames\n"
           << "    [-v|--verbose] print some info strings\n"
           << "\n"
           << "  Current open source EXIF writer libraries are GPL'd.\n"
           << "  We provide only the XML output that can be added using exiftool.\n"
           << "    [-e|--exif]    Print the FOV in XML suitable for exiftool\n"
           << endl;

      cerr << s << endl;
      return EXIT_FAILURE;
  }

  verbose = cmd.used('v');
  
  if( !stlplus::file_exists( inputVideoFilename ) )
  {
    cerr << "The video input file " << inputVideoFilename << " does not exist." << endl;
    return false;
  }

  // Check output directory
  if( stlplus::is_present( outputDirectory ) )
  {
    if( !stlplus::is_folder(outputDirectory ) )
    {
      cerr << "A file " << outputDirectory << " is present but it is not a directory." << endl;
      return false;
    }
  }
  else
  {
    if( !stlplus::folder_create( outputDirectory ) )
    {
      cerr << "Directory " << outputDirectory << " did not exist but creation failed." << endl;
      return false;
    }
  }

  cam_mode_360_t mode = stringToMode( modeString );

  img_format_t img_format = stringToImgFormat( imgFormatString );

  ConvertVideo cv( inputVideoFilename,
                   outputDirectory,
                   img_format,
                   mode,
                   cmd.used('e'),
                   cmd.used('s') );
  
  bool ret = cv.convert( );

  if( not ret )
  {
    cerr << "Failed to find codecs for any video stream in file " << inputVideoFilename << endl;
    return false;
  }


  return 0;
}


