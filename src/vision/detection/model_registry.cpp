#include "t170/vision/dl.h"

namespace vision {
void ModelRegistry::clear()
{
    entries_.clear();
}

void ModelRegistry::upsert(ModelEntry entry)
{
    const std::string key = entry.key;
    entries_[key] = std::move(entry);
}

void ModelRegistry::register_defaults(const std::string &head_weights, const std::string &hand_weights)
{
    upsert(ModelEntry{
        "tray_parts",
        "detect",
        head_weights,
        2,
        "料盘关联用；业务仍按 class 挂孔（S2 延期）",
    });
    upsert(ModelEntry{
        "belt_presence",
        "detect",
        head_weights,
        2,
        "传送带 ROI 有无+数量；不参与定位。与 pose_params segmentation.model 相同",
    });
    upsert(ModelEntry{
        "hand_parts",
        "detect",
        hand_weights,
        2,
        "左右手下降。与 pose_params segmentation.hand_model 相同，类别号与头相机一致",
    });
}

bool ModelRegistry::has(const std::string &key) const
{
    return entries_.find(key) != entries_.end();
}

const ModelEntry *ModelRegistry::get(const std::string &key) const
{
    const auto it = entries_.find(key);
    return it == entries_.end() ? nullptr : &it->second;
}

}
