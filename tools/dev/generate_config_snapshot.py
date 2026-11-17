"""Developer-only member serializer regeneration; needs clang++, not needed at runtime. Run at repository root."""
import re,subprocess
from pathlib import Path
r=Path('.');headers=['include/t170/common/config.h','include/t170/control/visual_servo.h','include/t170/common/tray_types.h']
types=[]
for h in headers:
 text=Path(h).read_text()
 names=re.findall(r'^struct (\w+)\s*\{',text,re.M)
 for name in names:
  if h.endswith('visual_servo.h') and name not in ['MapKnot','ImageProfile','PlacementCalibrationProfile','Config']:continue
  if h.endswith('tray_types.h') and name!='TrayPlaceVisionConfig':continue
  full=('wrist_servo::' if h!=headers[0] else '')+name
  ast=subprocess.check_output(['clang++','-std=c++20','-Iinclude','-Idependencies/libraries/eigen','-Xclang','-ast-dump','-Xclang','-ast-dump-filter='+full,'-fsyntax-only','-x','c++',headers[0]],stderr=subprocess.DEVNULL,text=True)
  fields=re.findall(r"^[|`]?-FieldDecl[^\n]*?\b(\w+) '([^']+)'",ast,re.M)
  # Clang's first-level members are prefixed '|-' or '`-'.
  if not fields:raise ValueError((name,ast[:200]))
  types.append((full,[n for n,t in fields]))
prefix='''// Explicit runtime member snapshot. Includes defaults, derived fields and loaded calibration.
#include "t170/config/effective.h"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <type_traits>
namespace t170::config {
namespace {
template<class T> requires (std::is_arithmetic_v<T> || std::is_same_v<T,std::string>)
YAML::Node encode(const T &v) {return YAML::Node(v);}
template<class D> YAML::Node encode(const Eigen::MatrixBase<D> &v) {
 YAML::Node n(YAML::NodeType::Sequence);
 for(int i=0;i<v.rows();++i) {YAML::Node row(YAML::NodeType::Sequence);for(int j=0;j<v.cols();++j)row.push_back(v(i,j));n.push_back(row);}return n;
}
'''
forward='\n'.join('YAML::Node encode(const '+t+' &);' for t,fs in types)
container='''
template<class T> YAML::Node encode(const std::vector<T> &v) {YAML::Node n(YAML::NodeType::Sequence);for(const auto &x:v)n.push_back(encode(x));return n;}
template<class T,std::size_t N> YAML::Node encode(const std::array<T,N> &v) {YAML::Node n(YAML::NodeType::Sequence);for(const auto &x:v)n.push_back(encode(x));return n;}
template<class T,std::size_t N> YAML::Node encode(const T (&v)[N]) {YAML::Node n(YAML::NodeType::Sequence);for(const auto &x:v)n.push_back(encode(x));return n;}
'''
body='\n'.join('YAML::Node encode(const '+t+' &v) {\n YAML::Node n;\n'+''.join(' n["'+f+'"] = encode(v.'+f+');\n' for f in fields)+' return n;\n}' for t,fields in types)
end='''
}
std::string effective_yaml(const MoveBoxConfig &cfg) {
 YAML::Emitter e;e.SetDoublePrecision(17);e<<encode(cfg);if(!e.good())throw std::runtime_error(e.GetLastError());return e.c_str();
}
}
'''
Path('src/config/effective.cpp').write_text(prefix+forward+container+body+end)
print('serialized types',len(types),'members',sum(len(fs) for t,fs in types))
