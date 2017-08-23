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

enum img_format_t
{
    JPG,
    PNG
};

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

class ConvertVideo
{
  const string         _filename;
  const string         _outputDirectory;
  const img_format_t   _img_format;
  const cam_mode_360_t _mode_360;
  const bool           _print_exif;
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
                bool print_exif )
    : _filename( filename )
    , _outputDirectory( outputDirectory )
    , _img_format( img_format )
    , _mode_360( mode )
    , _print_exif( print_exif )
    , ctx( 0 )
    , videoCtx( 0 )
    , videoCodec( 0 )
  {
    // register all codecs
    av_register_all();
  }

  bool convert( )
  {
    cout << "Trying to open file " << _filename.c_str() << endl;
    int err = avformat_open_input( &ctx, _filename.c_str(), 0, 0 );
    if( err )
    {
      err = av_strerror( err, errbuf, 1000 );
      if( err != 0 ) strcpy( errbuf, "Reason unknown" );
      cout << "Failed to open input file " << _filename << " -- Reason: " << errbuf << endl;
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
      cout << "Found " << ctx->nb_streams << " streams in " << _filename << endl;

      bool retval;
      for( int i=0; i<ctx->nb_streams; i++ )
      {
        retval = find_video( i );
        if( retval == true )
        {
          return true;
        }
      }
      return false;
    }

    err = av_strerror( err, errbuf, 1000 );
    if( err != 0 ) strcpy( errbuf, "Reason unknown" );
    cout << "Failed to find stream info in file " << _filename << " -- Reason: " << errbuf << endl;
    return false;
  }

  bool find_video( int i )
  {
    if( ctx->streams[i]->codec->codec_type == AVMEDIA_TYPE_VIDEO )
    {
      videoCtx = ctx->streams[i]->codec;
      if( !videoCtx )
      {
        cout << "Video stream " << i << " has type media but no video context" << endl;
        return false;
      }

      const AVCodecDescriptor* desc;
      desc = avcodec_descriptor_get( videoCtx->codec_id );
      if( desc )
      {
        cout << "Found codec " << desc->name << " for stream, " << desc->long_name << endl;
      }
      else
      {
        cout << "Found no codec descriptor for video stream " << i << endl;
        return false;
      }

      videoCodec = avcodec_find_decoder( videoCtx->codec_id );
      if( videoCodec )
      {
        return open_codec( i );
      }
      else
      {
        cout << "Found video codec descriptor but not codec for stream " << i << endl;
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

      cout << "Detect frames with WxH " << _width << "x" << _height << endl;
      cout << "Stream has " << fps << " fps and " << duration << " second duration" << endl;

      bool retval = read_data( videoStreamIndex );

      avcodec_close( videoCtx );
      return retval;
    }
        
    err = av_strerror( err, errbuf, 1000 );
    if( err != 0 ) strcpy( errbuf, "Reason unknown" );
    cout << "Failed to open video codec for stream -- Reason: " << errbuf << endl;

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
      cout << "Got no frame from context -- Reason: " << errbuf << endl;
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
          cout << "Got no frame data from packet -- Reason: " << errbuf << endl;
        }
        else if( videoFrameBytes > 0 )
        {
          if( got_picture != 0 )
          {
            cout << "Frame " << numbering << endl;
#undef DUMP_IN_FILE
#ifdef DUMP_IN_FILE
            {
                ostringstream name;
                name << _outputDirectory << "/" << "in-file-" << numbering << ".txt";
                ofstream of( name.str() );
                of << "Width = " << _width << endl;
                of << "Height = " << _height << endl;
                for( int i=0; i<AV_NUM_DATA_POINTERS; i++ ) {
                    of << "Linesize " << i << " : " << frame->linesize[i] << endl;
                }

                for( int h=0; h<_height; h++ ) {
                    for( int w=0; w<_width; w+=1 ) {
                        of << int(frame->data[0][h*frame->linesize[0]+w]) << ","
                           << int(frame->data[1][h/2*frame->linesize[1]+w/2]) << ","
                           << int(frame->data[2][h/2*frame->linesize[2]+w/2]) << " ";
                    }
                    of << endl;
                }
            }
#endif // DUMP_IN_FILE

            // AVPicture contains only the first 2 members of AVFrame
            avpicture_fill((AVPicture*)outframe, out_buffer, AV_PIX_FMT_RGB24, _width, _height );

            AVPixelFormat             fmt  = (AVPixelFormat)frame->format;
            const AVPixFmtDescriptor* desc = av_pix_fmt_desc_get( fmt );
            // cout << "(" << __LINE__ << ") new picture - read " << videoFrameBytes << " bytes" << endl;
            // cout << "Frame. pixfmt=" << av_get_pix_fmt_name( fmt ) << " num planes=" << av_pix_fmt_count_planes( fmt ) << " bits per pixel=" << av_get_bits_per_pixel( desc ) << endl;

            int height = sws_scale( convertCtx,
                                    frame->data, 
                                    frame->linesize, 0, 
                                    videoCtx->height,
                                    outframe->data,
                                    outframe->linesize ); 
            // cout << "result height of sws_scale: " << height << endl;

#undef DUMP_OUT_FILE
#ifdef DUMP_OUT_FILE
            {
                ostringstream name;
                name << _outputDirectory << "/" << "out-file-" << numbering << ".txt";
                ofstream of( name.str() );
                of << "Width = " << _width << endl;
                of << "Height = " << _height << endl;
                for( int i=0; i<AV_NUM_DATA_POINTERS; i++ ) {
                    of << "Linesize " << i << " : " << outframe->linesize[i] << endl;
                }

                int      linesize = outframe->linesize[0];
                uint8_t* data     = outframe->data[0];

                for( int h=0; h<_height; h++ ) {
                    for( int w=0; w<_width*3; w+=3 ) {
                        of << int(data[h*linesize+w+0]) << ","
                           << int(data[h*linesize+w+1]) << ","
                           << int(data[h*linesize+w+2]) << " ";

                        float y = frame->data[0][h*frame->linesize[0]+w];
                        float u = frame->data[1][h/2*frame->linesize[1]+w/2];
                        float v = frame->data[2][h/2*frame->linesize[2]+w/2];
                        float r = (int)(y + 1.402 * ( u - 128.0 ) );
                        float g = (int)(y - 0.344 * ( v - 128.0 ) - 0.714 * ( u - 128.0 ) );
                        float b = (int)(y + 1.772 * ( v - 128.0 ) );
                        of << "(" << int(r) << "," << int(g) << "," << int(b) << ") ";
                    }
                    of << endl;
                }
            }
#endif // DUMP_OUT_FILE

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

            ostringstream filebase;
            ostringstream filename;
            ostringstream fullname;
            filebase << "file-" << numbering;
            filename << filebase.str() << (_img_format==img_format_t::PNG ? ".png" : ".jpg" );
            fullname << _outputDirectory << "/" << filename.str();
            int err = openMVG::image::WriteImage( fullname.str().c_str(), out );

            if( _print_exif )
            {
              if( _mode_360 == cam_mode_360_t::EQUIRECT || _mode_360 == cam_mode_360_t::CUBEMAP )
              {
                int deg = ( _mode_360 == cam_mode_360_t::EQUIRECT ) ? 360 : 90;
                ostringstream xmlname;
                xmlname << _outputDirectory << "/" << filebase.str() << ".rdf";
                ofstream rdfout( xmlname.str() );
                rdfout << "<?xml version='1.0' encoding='UTF-8'?>\n"
                       << "<rdf:RDF xmlns:rdf='http://www.w3.org/1999/02/22-rdf-syntax-ns#'>\n"
                       << "  <rdf:Description rdf:about='" << fullname.str() << "'>\n"
                       << "    <System:FileName>" << filename.str() << "</System:FileName>\n"
                       << "    <System:Directory>" << _outputDirectory << "</System:Directory>\n"
                       << "    <Composite:FOV>" << deg << " deg</Composite:FOV>\n"
                       << "  </rdf:Description>\n"
                       << "</rdf:RDF>\n";
              }
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
  
  try
  {
      if (argc == 1) throw std::string("Invalid command line parameter.");
      cmd.process(argc, argv);
  }
  catch(const std::string& s)
  {
      std::cerr << "Usage: " << argv[0] << '\n'
                << "    [-i|--input]  the input HEVC video file\n"
                << "    [-o|--output] the directory output image should be stored\n"
                << "    [-m|--mode]   EQUIRECT if the input is an equirectangular 360 video\n"
                << "                  CUBEMAP  if the input is a CubeMap 360 video\n"
                << "    [-f|--format] output image format, either PNG (default) or JPG\n"
                << "  Current open source EXIF writer libraries are GPL'd.\n"
                << "  We provide only the XML output that can be added using exiftool.\n"
                << "    [-e|--exif] Print the FOV in XML suitable for exiftool.\n"
                << std::endl;

      std::cerr << s << std::endl;
      return EXIT_FAILURE;
  }
  
  if( !stlplus::file_exists( inputVideoFilename ) )
  {
    std::cout << "The video input file " << inputVideoFilename << " does not exist." << std::endl;
    return false;
  }

  // Check output directory
  if( stlplus::is_present( outputDirectory ) )
  {
    if( !stlplus::is_folder(outputDirectory ) )
    {
      std::cout << "A file " << outputDirectory << " is present but it is not a directory." << std::endl;
      return false;
    }
  }
  else
  {
    if( !stlplus::folder_create( outputDirectory ) )
    {
      std::cout << "Directory " << outputDirectory << " did not exist but creation failed." << std::endl;
      return false;
    }
  }

  cam_mode_360_t mode = stringToMode( modeString );

  img_format_t img_format = stringToImgFormat( imgFormatString );

  ConvertVideo cv( inputVideoFilename, outputDirectory, img_format, mode, cmd.used('e') );
  
  bool ret = cv.convert( );

  if( not ret )
  {
    cout << "Failed to find codecs for any video stream in file " << inputVideoFilename << endl;
    return false;
  }


  return 0;
}


