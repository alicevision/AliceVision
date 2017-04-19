#include "openMVG/image/image.hpp"
#include "openMVG/sfm/sfm.hpp"

#include <cereal/archives/json.hpp>
#include "openMVG/system/timer.hpp"

#include "third_party/cmdLine/cmdLine.h"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"
#include "third_party/progress/progress.hpp"

#include <cstdlib>
#include <fstream>
#include <string>

using namespace std;
using namespace openMVG;
using namespace openMVG::image;
using namespace openMVG::features;
using namespace openMVG::sfm;
/*
 * 
 */
int main(int argc, char** argv) {

  CmdLine cmd;
  
  std::string sSfM_Data_Filename;
  std::string directory;
  
  cmd.add( make_option('i', sSfM_Data_Filename, "input_file") );
  cmd.add( make_option('d', directory, "directory") );
  
  try {
      if (argc == 1) throw std::string("Invalid command line parameter.");
      cmd.process(argc, argv);
  } catch(const std::string& s)
  {
      std::cerr << "Usage: " << argv[0] << '\n'
      << "[-i|--input_file] the sfm_data file\n"
      << "[-d|--directory] The directory where .feat and .desc files are\n"
      << std::endl;

      std::cerr << s << std::endl;
      return EXIT_FAILURE;
  }
  
  //Check directory name
  if (!stlplus::is_folder(directory))
  {
    std::cout << "The directory can't be found" << std::endl;
    return false;
  }
  
  //Load Smf_data file:
  SfM_Data sfm_data;
  if (!Load(sfm_data, sSfM_Data_Filename, ESfM_Data(VIEWS|INTRINSICS))) {
    std::cerr << std::endl
      << "The input file \""<< sSfM_Data_Filename << "\" cannot be read" << std::endl;
    return false;
  }
  
  Views::const_iterator iterViews = sfm_data.views.begin();
  Views::const_iterator iterViewsEnd = sfm_data.views.end();
  
  for(; iterViews != iterViewsEnd; ++iterViews)
  {
    const View * view = iterViews->second.get();
    
    std::string filename;
    std::string newname;
    std::string compared2(1,'/');
    std::string id = to_string(view->id_view);
            
    if (directory.substr(directory.size() - 1, directory.size()).compare(compared2) == 0)
    {
      filename = directory + view->s_Img_path.substr(1 ,view->s_Img_path.find('.'));
      newname = directory + id;
    }
    else 
    {
      filename = directory + view->s_Img_path.substr(0 ,view->s_Img_path.find('.') + 1);
      newname = directory + compared2 + id;
    }
   
     
    
    std::string oldDesc = filename + "desc";
    std::string newDesc = newname + ".desc";
    
    std::string oldFeat = filename + "feat";
    std::string newFeat = newname + ".feat";
    
    if (rename(oldDesc.c_str(), newDesc.c_str()) != 0)
      std::cout<< "Cannot rename" << oldDesc <<  std::endl;
    
    if (rename(oldFeat.c_str(), newFeat.c_str()) != 0)
      std::cout<< "Cannot rename" << oldFeat <<  std::endl;
      
  }
  
  return 0;
}


