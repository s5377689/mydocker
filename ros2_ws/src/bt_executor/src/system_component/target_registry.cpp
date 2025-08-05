#include "system_component/target_registry.hpp"

namespace system_component
{

void TargetRegistry::register_target(
    const std::string & id,
    const TargetData & data)
{
    std::lock_guard<std::mutex> lock(mutex_);
    registry_[id] = data;
}

bool TargetRegistry::get_target(
    const std::string & id,
    TargetData & out_data)
{
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = registry_.find(id);
    if (it != registry_.end())
    {
        out_data = it->second;
        return true;
    }
    return false;
}

std::vector<std::string> TargetRegistry::get_target_list()
{
    std::lock_guard<std::mutex> lock(mutex_);

    std::vector<std::string> ids;
    ids.reserve(registry_.size());

    for (const auto & pair : registry_)
    {
        ids.push_back(pair.first);
    }
    return ids;
}

bool TargetRegistry::has_target(
    const std::string & id)
{
    std::lock_guard<std::mutex> lock(mutex_);
    return registry_.find(id) != registry_.end();
}

void TargetRegistry::remove_target(
    const std::string & id)
{
    std::lock_guard<std::mutex> lock(mutex_);
    registry_.erase(id);
}

void TargetRegistry::clear_all_targets()
{
    std::lock_guard<std::mutex> lock(mutex_);
    registry_.clear();
}

}  // namespace system_component