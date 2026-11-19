#include "t170/app/commands.h"
#include "t170/app/options.h"
#include <iostream>
#include <set>
#include <stdexcept>
#include <vector>
namespace {
void require(bool ok,const char *message){if(!ok)throw std::runtime_error(message);}
t170::app::Options parse(std::initializer_list<const char *> args){std::vector<const char *> values(args);return t170::app::parse_options(int(values.size()),values.data());}
void rejects(std::initializer_list<const char *> args){try{parse(args);}catch(const std::invalid_argument &){return;}throw std::runtime_error("invalid CLI input accepted");}
}
int main(int argc,char **) {
    using namespace t170::app;
    try {
        if(argc>1) {
            std::cout<<'[';bool first=true;
            for(const auto &command:command_catalog()) {
                if(!first)std::cout<<',';first=false;
                std::cout<<"{\"name\":\""<<command.name<<"\",\"group\":\""<<group_name(command.group)
                         <<"\",\"production\":"<<(command.production?"true":"false")<<'}';
            }std::cout<<"]\n";return 0;
        }
        require(parse({"app"}).mode==Mode::Maintenance,"legacy no-args entry must remain manual");
        require(parse({"app","--production"}).mode==Mode::Production,"production entry not selected");
        require(parse({"app","--dispatch"}).mode==Mode::Production,"old --dispatch alias broke");
        require(parse({"app","--maintenance","--port","8123"}).port==8123,"port parsing failed");
        require(parse({"app","--production","--help"}).help,"help cannot be requested in production mode");
        rejects({"app","--maintenance","--production"});rejects({"app","--port"});
        for(const char *value:{"0","65536","-1","x","8099x"})rejects({"app","--port",value});
        rejects({"app","--unknown"});
        const std::set<std::string_view> allowed={"ping","abort","dispatch_exit","dispatch_status",
            "reload_config","snapshot","dispatch","capabilities","calibration_status"};
        std::set<std::string_view> seen;
        for(const auto &spec:command_catalog()) {
            require(seen.insert(spec.name).second,"duplicate command specification");
            require(command_allowed(Mode::Maintenance,spec.name)==(spec.group!=CommandGroup::Retired),"maintenance retirement policy drifted");
            require(command_allowed(Mode::Production,spec.name)==allowed.contains(spec.name),"production policy drifted");
        }
        require(!command_allowed(Mode::Maintenance,"typo") && !command_allowed(Mode::Production,"placement_calibration"),
                "unknown command or production calibration admitted");
        require(!command_collects_arrivals("capabilities") && !command_collects_arrivals("dispatch_status") &&
                !command_collects_arrivals("dispatch_exit") && !command_collects_arrivals("typo") &&
                command_collects_arrivals("vision_grasp"),"status polling can erase active motion arrivals");
        std::cout<<"application policy: explicit modes, old alias, invalid CLI, command coverage and production access passed\n";
    }catch(const std::exception &ex){std::cerr<<ex.what()<<'\n';return 1;}
}
