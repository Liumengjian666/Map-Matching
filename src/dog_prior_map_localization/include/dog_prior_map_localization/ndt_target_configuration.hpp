#pragma once

namespace dog_prior_map_localization
{
// PCL constructs its Gaussian cells in setInputTarget().  With no Source yet,
// a later setResolution() changes the parameter but does not rebuild the cells.
// Use the same order for the first global target and every local-map rebuild.
template <typename Ndt, typename TargetPtr>
void setNdtTargetAtResolution(Ndt &ndt, const TargetPtr &target, float resolution)
{
  ndt.setResolution(resolution);
  ndt.setInputTarget(target);
}
}  // namespace dog_prior_map_localization
