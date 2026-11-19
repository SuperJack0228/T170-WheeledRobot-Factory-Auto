#include "t170/config/effective.h"
#include <fstream>
#include <iostream>
int main(int argc,char **argv) {
 try {
  const bool compare=argc==4 && std::string(argv[1])=="--compare";
  if((!compare && argc!=3)) {std::cerr<<"Usage: t170_config_dump INPUT OUTPUT | --compare BEFORE AFTER\n";return 2;}
  MoveBoxConfig a,b;std::string error;
  if(!load_move_box_config(argv[compare?2:1],a,error))throw std::runtime_error(error);
  const auto first=t170::config::effective_yaml(a);
  if(compare) {
   if(!load_move_box_config(argv[3],b,error))throw std::runtime_error(error);
   if(first!=t170::config::effective_yaml(b)) {std::cerr<<"Effective configurations differ\n";return 1;}
   std::cout<<"All loaded configuration members are equivalent\n";
  } else {std::ofstream out(argv[2]);out<<first<<'\n';if(!out)throw std::runtime_error("snapshot write failed");}
  return 0;
 } catch(const std::exception &e) {std::cerr<<e.what()<<'\n';return 2;}
}
