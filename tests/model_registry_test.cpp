#include "t170/vision/dl.h"
#include <iostream>
int main() {
    vision::ModelRegistry registry;
    registry.register_defaults("/site/models/head-v7.pt", "/site/models/wrist-v9.pt");
    if(registry.get("tray_parts")->weights!="/site/models/head-v7.pt" ||
       registry.get("belt_presence")->weights!="/site/models/head-v7.pt" ||
       registry.get("hand_parts")->weights!="/site/models/wrist-v9.pt") return 1;
    registry.clear();registry.register_defaults();
    if(!registry.get("hand_parts")->weights.empty()) return 1;
    std::cout<<"model registry: configured engine identities and unbound state passed (no weights loaded)\n";
}
