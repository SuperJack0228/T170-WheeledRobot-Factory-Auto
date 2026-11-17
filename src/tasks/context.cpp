#include "t170/tasks/context.h"
#include <stdexcept>
namespace t170::tasks {
namespace {std::atomic<Services *> bound{nullptr};}
void bind_services(Services *services){bound.store(services);}
Services &services(){auto *value=bound.load();if(!value)throw std::logic_error("factory services not assembled");return *value;}
}
