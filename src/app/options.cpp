#include "t170/app/options.h"
#include <charconv>
#include <stdexcept>
#include <string_view>
namespace t170::app {
const char *mode_name(Mode mode){return mode==Mode::Production?"production":"maintenance";}
const char *usage(){return
    "t170_factory [--production | --maintenance] [--port 8099]\n"
    "  --production  Initialize and enter automatic dispatch; operational commands only.\n"
    "  --maintenance Initialize without automatic dispatch; calibration/manual diagnostics enabled.\n"
    "  --dispatch    Compatibility alias for --production.\n"
    "  --list-commands  Print the command catalog without hardware initialization.\n"
    "  --help           Print this help without hardware initialization.\n";
}
Options parse_options(int argc,const char *const *argv) {
    Options out;bool explicit_mode=false;
    for(int i=1;i<argc;++i) {
        const std::string_view arg=argv[i];
        if(arg=="--production" || arg=="--dispatch" || arg=="--maintenance") {
            const Mode next=arg=="--maintenance"?Mode::Maintenance:Mode::Production;
            if(explicit_mode && out.mode!=next)throw std::invalid_argument("production and maintenance modes cannot be combined");
            out.mode=next;explicit_mode=true;
        } else if(arg=="--port") {
            if(++i>=argc)throw std::invalid_argument("--port requires a value");
            const std::string_view value=argv[i];int port=0;
            const auto parsed=std::from_chars(value.data(),value.data()+value.size(),port);
            if(parsed.ec!=std::errc{} || parsed.ptr!=value.data()+value.size() || port<1 || port>65535)
                throw std::invalid_argument("port must be an integer in 1..65535");
            out.port=port;
        } else if(arg=="--help" || arg=="-h")out.help=true;
        else if(arg=="--list-commands")out.list_commands=true;
        else throw std::invalid_argument("unknown argument: "+std::string(arg));
    }return out;
}
}
