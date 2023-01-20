#pragma once

#include <opencv2/core/core.hpp>

namespace dvs_displayer
{

void seismic_cmap(cv::Mat& lut);
void viridis_cmap(cv::Mat& lut);

} // namespace
