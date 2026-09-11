#include "dog_prior_map_localization/corridor_sequence_localizer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace dog_prior_map_localization
{

void CorridorSequenceLocalizer::configure(bool enabled, double bin_size, int sequence_length,
                                           int max_hypotheses, double search_radius,
                                           double axis_min, double axis_max)
{
  enabled_ = enabled;
  bin_size_ = std::max(0.1, bin_size);
  sequence_length_ = std::max(1, sequence_length);
  max_hypotheses_ = std::max(1, max_hypotheses);
  search_radius_ = std::max(bin_size_, search_radius);
  axis_min_ = axis_min;
  axis_max_ = std::max(axis_min_ + bin_size_, axis_max);
  descriptor_bins_ = longitudinal_bins_ * lateral_bins_ * height_bins_;
  ready_ = false;
  map_descriptors_.clear();
  observation_history_.clear();
  hypotheses_.clear();
  has_last_prediction_ = false;
  last_predicted_s_ = 0.0;
}

void CorridorSequenceLocalizer::buildMap(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &map)
{
  if (!enabled_ || !map || map->empty()) return;
  if (axis_max_ <= axis_min_ + bin_size_)
  {
    axis_min_ = std::numeric_limits<double>::max();
    axis_max_ = std::numeric_limits<double>::lowest();
    for (const auto &p : map->points)
    {
      if (!std::isfinite(p.x)) continue;
      axis_min_ = std::min(axis_min_, static_cast<double>(p.x));
      axis_max_ = std::max(axis_max_, static_cast<double>(p.x));
    }
  }
  const int count = std::max(1, static_cast<int>(std::ceil((axis_max_ - axis_min_) / bin_size_)) + 1);
  map_descriptors_.assign(static_cast<size_t>(count), Descriptor::Zero(descriptor_bins_));
  for (const auto &p : map->points)
  {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
    const int center = static_cast<int>(std::floor((p.x - axis_min_) / bin_size_));
    if (center < 0 || center >= count) continue;
    for (int li = 0; li < longitudinal_bins_; ++li)
    {
      const double dx = (p.x - (axis_min_ + (center + 0.5) * bin_size_)) / bin_size_;
      const int expected = static_cast<int>(std::floor(dx + longitudinal_bins_ / 2.0));
      if (expected != li) continue;
      const int yi = std::max(0, std::min(lateral_bins_ - 1, static_cast<int>(std::floor(p.y + lateral_bins_ / 2.0))));
      const int zi = std::max(0, std::min(height_bins_ - 1, static_cast<int>(std::floor(p.z + 1.0))));
      map_descriptors_[static_cast<size_t>(center)](li * lateral_bins_ * height_bins_ + yi * height_bins_ + zi) += 1.0f;
      break;
    }
  }
  for (auto &d : map_descriptors_)
  {
    const float n = d.norm();
    if (n > 1e-6f) d /= n;
  }
  ready_ = map_descriptors_.size() > 3;
}

CorridorSequenceLocalizer::Descriptor CorridorSequenceLocalizer::descriptorForMapBin(int index) const
{
  if (index < 0 || index >= static_cast<int>(map_descriptors_.size())) return Descriptor::Zero(descriptor_bins_);
  return map_descriptors_[static_cast<size_t>(index)];
}

CorridorSequenceLocalizer::Descriptor CorridorSequenceLocalizer::descriptorForObservation(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &scan, const Eigen::Matrix3d &R,
    const Eigen::Vector3d &pose, double candidate_s) const
{
  Descriptor d = Descriptor::Zero(descriptor_bins_);
  if (!scan) return d;
  for (const auto &pt : scan->points)
  {
    Eigen::Vector3d pw = R * Eigen::Vector3d(pt.x, pt.y, pt.z) + pose;
    if (!pw.allFinite()) continue;
    const int li = static_cast<int>(std::floor((pw.x() - candidate_s) / bin_size_ + longitudinal_bins_ / 2.0));
    if (li < 0 || li >= longitudinal_bins_) continue;
    const int yi = std::max(0, std::min(lateral_bins_ - 1, static_cast<int>(std::floor(pw.y() + lateral_bins_ / 2.0))));
    const int zi = std::max(0, std::min(height_bins_ - 1, static_cast<int>(std::floor(pw.z() + 1.0))));
    d(li * lateral_bins_ * height_bins_ + yi * height_bins_ + zi) += 1.0f;
  }
  const float n = d.norm();
  if (n > 1e-6f) d /= n;
  return d;
}

double CorridorSequenceLocalizer::descriptorDistance(const Descriptor &a, const Descriptor &b) const
{
  if (a.size() == 0 || b.size() != a.size()) return 1.0;
  return std::max(0.0, 1.0 - static_cast<double>(a.dot(b)));
}

void CorridorSequenceLocalizer::normalizeHypotheses()
{
  double total = 0.0;
  for (const auto &h : hypotheses_) total += std::max(0.0, h.probability);
  if (total <= 1e-12) return;
  for (auto &h : hypotheses_) h.probability = std::max(0.0, h.probability) / total;
  std::sort(hypotheses_.begin(), hypotheses_.end(), [](const Hypothesis &a, const Hypothesis &b) {
    return a.probability > b.probability;
  });
  if (static_cast<int>(hypotheses_.size()) > max_hypotheses_) hypotheses_.resize(static_cast<size_t>(max_hypotheses_));
  total = 0.0;
  for (const auto &h : hypotheses_) total += h.probability;
  for (auto &h : hypotheses_) h.probability /= std::max(total, 1e-12);
}

double CorridorSequenceLocalizer::update(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &scan,
                                         const Eigen::Matrix3d &R, const Eigen::Vector3d &pose,
                                         double predicted_s, bool degenerate)
{
  if (!ready_ || !scan || scan->empty()) return pose.x();
  const double center = predicted_s;
  const double motion_delta = has_last_prediction_ ? predicted_s - last_predicted_s_ : 0.0;
  const int first = std::max(0, static_cast<int>(std::floor((center - search_radius_ - axis_min_) / bin_size_)));
  const int last = std::min(static_cast<int>(map_descriptors_.size()) - 1,
                            static_cast<int>(std::ceil((center + search_radius_ - axis_min_) / bin_size_)));
  std::vector<Hypothesis> candidates;
  for (int i = first; i <= last; ++i)
  {
    const double s = axis_min_ + (i + 0.5) * bin_size_;
    const Descriptor obs = descriptorForObservation(scan, R, pose, s);
    double score = descriptorDistance(obs, descriptorForMapBin(i));
    // Propagate every previous scalar hypothesis with the IMU/NDT predicted
    // longitudinal motion. This is the temporal multi-hypothesis prior.
    double prior = hypotheses_.empty() ? 1.0 : 0.0;
    for (const auto &h : hypotheses_)
    {
      const double propagated = h.s + motion_delta;
      prior += h.probability * std::exp(-0.5 * std::pow((s - propagated) / std::max(bin_size_, 0.25), 2.0));
    }
    score = 0.65 * score + 0.35 * std::abs(s - predicted_s) / search_radius_;
    candidates.push_back({s, std::max(1e-9, prior) * std::exp(-4.0 * score)});
  }
  if (candidates.empty()) return pose.x();
  hypotheses_ = std::move(candidates);
  normalizeHypotheses();
  const double best = hypotheses_.front().s;
  last_score_ = hypotheses_.front().probability;
  observation_history_.push_back(descriptorForObservation(scan, R, pose, best));
  while (static_cast<int>(observation_history_.size()) > sequence_length_) observation_history_.pop_front();
  has_last_prediction_ = true;
  last_predicted_s_ = predicted_s;
  // During observable frames, follow NDT; during degeneracy, retain the
  // sequence estimate and only allow a bounded transition per frame.
  if (!degenerate) return pose.x();
  const double max_step = std::max(bin_size_, 2.0 * bin_size_);
  return pose.x() + std::max(-max_step, std::min(max_step, best - pose.x()));
}

}  // namespace dog_prior_map_localization
