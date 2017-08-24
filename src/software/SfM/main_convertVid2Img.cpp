#include "openMVG/image/image.hpp"
#include "openMVG/image/pixel_types.hpp"

// #include "openMVG/system/timer.hpp"

#include "third_party/cmdLine/cmdLine.h"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"
// #include "third_party/progress/progress.hpp"

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
static char cam_cube_order[7] = "RLUDFB";

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

class ConvertVideo
{
  const string         _filename;
  const string         _outputDirectory;
  const img_format_t   _img_format;
  const cam_mode_360_t _mode_360;
  const bool           _print_exif;
  const bool           _split_cube;
  cam_cube_conf_t      _conf_360;
  char                 errbuf[1000];
  AVFormatContext*     ctx;
  AVCodecContext*      videoCtx;
  AVCodec*             videoCodec;
  int                  _width;
  int                  _height;
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


