#include "yolov8.h"
#include <opencv2/cudaimgproc.hpp>


// Custom NMS implementation
static void nonMaxSuppression(
    const std::vector<cv::Rect_<float>>& boxes,
    const std::vector<float>& scores,
    const std::vector<int>& labels,
    float scoreThreshold,
    float nmsThreshold,
    std::vector<int>& indices) {
    
    // Get indices of all boxes with score > threshold
    std::vector<std::pair<float, int>> score_index_pairs;
    for (size_t i = 0; i < scores.size(); ++i) {
        if (scores[i] > scoreThreshold) {
            score_index_pairs.emplace_back(scores[i], i);
        }
    }

    // Sort by score in descending order
    std::sort(score_index_pairs.begin(), score_index_pairs.end(),
              [](const auto& a, const auto& b) { return a.first > b.first; });

    indices.clear();
    std::vector<bool> suppressed(boxes.size(), false);

    // Perform NMS
    for (size_t i = 0; i < score_index_pairs.size(); ++i) {
        if (suppressed[score_index_pairs[i].second]) continue;

        indices.push_back(score_index_pairs[i].second);

        // Compare with all remaining boxes
        for (size_t j = i + 1; j < score_index_pairs.size(); ++j) {
            if (suppressed[score_index_pairs[j].second]) continue;

            // Only suppress boxes with same class label
            if (labels[score_index_pairs[i].second] != labels[score_index_pairs[j].second]) continue;

            const auto& box1 = boxes[score_index_pairs[i].second];
            const auto& box2 = boxes[score_index_pairs[j].second];

            // Calculate intersection over union (IoU)
            float intersectX1 = std::max(box1.x, box2.x);
            float intersectY1 = std::max(box1.y, box2.y);
            float intersectX2 = std::min(box1.x + box1.width, box2.x + box2.width);
            float intersectY2 = std::min(box1.y + box1.height, box2.y + box2.height);

            float intersectArea = std::max(0.0f, intersectX2 - intersectX1) *
                                std::max(0.0f, intersectY2 - intersectY1);
            float box1Area = box1.width * box1.height;
            float box2Area = box2.width * box2.height;
            float iou = intersectArea / (box1Area + box2Area - intersectArea);

            if (iou > nmsThreshold) {
                suppressed[score_index_pairs[j].second] = true;
            }
        }
    }
}

YoloV8::YoloV8(const std::string &onnxModelPath, const YoloV8Config &config)
    : PROBABILITY_THRESHOLD(config.probabilityThreshold), NMS_THRESHOLD(config.nmsThreshold), TOP_K(config.topK),
      SEG_CHANNELS(config.segChannels), SEG_H(config.segH), SEG_W(config.segW), SEGMENTATION_THRESHOLD(config.segmentationThreshold),
      CLASS_NAMES(config.classNames), NUM_KPS(config.numKPS), KPS_THRESHOLD(config.kpsThreshold) {
    Options options;
    options.optBatchSize = 1;
    options.maxBatchSize = 1;
    options.precision = config.precision;
    options.calibrationDataDirectoryPath = config.calibrationDataDirectory;

    if (options.precision == Precision::INT8) {
        if (options.calibrationDataDirectoryPath.empty()) {
            throw std::runtime_error("Error: Must supply calibration data path for INT8 calibration");
        }
    }

    m_trtEngine = std::make_unique<Engine<float>>(options);
    auto succ = m_trtEngine->buildLoadNetwork(onnxModelPath, SUB_VALS, DIV_VALS, NORMALIZE);
    if (!succ) {
        throw std::runtime_error("Error: Unable to build or load the TensorRT engine.");
    }
}

// Define threshold for keypoint confidence
static constexpr float KPS_THRESHOLD = 0.75f;

float YoloV8::estimateDistance(const cv::Rect_<float>& bbox, const std::vector<float>& keypoints) {
    // Check if we have enough keypoints to calculate shoulder width
    if (!keypoints.empty() && keypoints.size() >= 21) {
        // Extract shoulder positions and confidence scores
        float shoulder1X = keypoints[12];    // Shoulder X position (left shoulder)
        float shoulder1Y = keypoints[13];    // Shoulder Y position (left shoulder)
        float shoulder2X = keypoints[15];    // Other shoulder X position (right shoulder)
        float shoulder2Y = keypoints[16];    // Other shoulder Y position (right shoulder)
        float shoulder1Conf = keypoints[14]; // Confidence of the left shoulder
        float shoulder2Conf = keypoints[17]; // Confidence of the right shoulder

        // Ensure both shoulders are detected with at least 75% confidence
        if (shoulder1Conf > KPS_THRESHOLD && shoulder2Conf > KPS_THRESHOLD) {
            float leftShoulderX = std::min(shoulder1X, shoulder2X);
            float rightShoulderX = std::max(shoulder1X, shoulder2X);
            float leftShoulderY = std::min(shoulder1Y, shoulder2Y);
            float rightShoulderY = std::max(shoulder1Y, shoulder2Y);

            // Calculate shoulder width and height
            float shoulderWidth = rightShoulderX - leftShoulderX;
            float shoulderHeight = rightShoulderY - leftShoulderY;

            // Calculate the angle of the person's orientation
            float orientation = std::atan2(shoulderHeight, shoulderWidth);

            // Use shoulder width, height, and orientation to estimate distance
            if (shoulderWidth > 0) {
                if (std::abs(orientation) < 0.785398) { // Person is facing front (within 45 degrees)
                    return (AVERAGE_SHOULDER_WIDTH * CAMERA_FOCAL_LENGTH_SHOULDERS) / shoulderWidth;
                } else { // Person is oriented at an angle
                    float adjustedShoulderWidth = shoulderWidth / std::cos(orientation);
                    float adjustedShoulderHeight = shoulderHeight / std::sin(orientation);
                    float averageAdjustedSize = (adjustedShoulderWidth + adjustedShoulderHeight) / 2.0f;
                    return (AVERAGE_SHOULDER_WIDTH * CAMERA_FOCAL_LENGTH_SHOULDERS) / averageAdjustedSize;
                }
            }
        }
    }

    // Fallback to bbox.width if shoulders are unavailable or unreliable
    return (AVERAGE_PERSON_WIDTH * CAMERA_FOCAL_LENGTH) / bbox.width;
}



std::vector<std::vector<cv::cuda::GpuMat>> YoloV8::preprocess(const cv::cuda::GpuMat &gpuImg) {
    const auto &inputDims = m_trtEngine->getInputDims();

    cv::cuda::GpuMat rgbMat;
    cv::cuda::cvtColor(gpuImg, rgbMat, cv::COLOR_BGR2RGB);

    auto resized = rgbMat;
    if (resized.rows != inputDims[0].d[1] || resized.cols != inputDims[0].d[2]) {
        resized = Engine<float>::resizeKeepAspectRatioPadRightBottom(rgbMat, inputDims[0].d[1], inputDims[0].d[2]);
    }

    std::vector<cv::cuda::GpuMat> input{std::move(resized)};
    std::vector<std::vector<cv::cuda::GpuMat>> inputs{std::move(input)};

    m_imgHeight = rgbMat.rows;
    m_imgWidth = rgbMat.cols;
    m_ratio = 1.f / std::min(inputDims[0].d[2] / static_cast<float>(rgbMat.cols),
                            inputDims[0].d[1] / static_cast<float>(rgbMat.rows));

    return inputs;
}

std::vector<Object> YoloV8::detectObjects(const cv::cuda::GpuMat &inputImageBGR) {
    const auto input = preprocess(inputImageBGR);
    
    std::vector<std::vector<std::vector<float>>> featureVectors;
    auto succ = m_trtEngine->runInference(input, featureVectors);
    if (!succ) {
        throw std::runtime_error("Error: Unable to run inference.");
    }

    const auto &numOutputs = m_trtEngine->getOutputDims().size();
    std::vector<Object> ret;
    
    if (numOutputs == 1) {
        std::vector<float> featureVector;
        Engine<float>::transformOutput(featureVectors, featureVector);

        const auto &outputDims = m_trtEngine->getOutputDims();
        int numChannels = outputDims[outputDims.size() - 1].d[1];
        
        if (numChannels == 56) {
            ret = postprocessPose(featureVector);
        } else {
            ret = postprocessDetect(featureVector);
        }
    } else {
        std::vector<std::vector<float>> featureVector;
        Engine<float>::transformOutput(featureVectors, featureVector);
        ret = postProcessSegmentation(featureVector);
    }

    return ret;
}

std::vector<Object> YoloV8::detectObjects(const cv::Mat &inputImageBGR) {
    cv::cuda::GpuMat gpuImg;
    gpuImg.upload(inputImageBGR);
    return detectObjects(gpuImg);
}

std::vector<Object> YoloV8::postprocessPose(std::vector<float> &featureVector) {
    const auto &outputDims = m_trtEngine->getOutputDims();
    auto numChannels = outputDims[0].d[1];
    auto numAnchors = outputDims[0].d[2];

    std::vector<cv::Rect_<float>> bboxes;
    std::vector<float> scores;
    std::vector<int> labels;
    std::vector<int> indices;
    std::vector<std::vector<float>> kpss;

    cv::Mat output = cv::Mat(numChannels, numAnchors, CV_32F, featureVector.data());
    output = output.t();

    for (int i = 0; i < numAnchors; i++) {
        auto rowPtr = output.row(i).ptr<float>();
        auto bboxesPtr = rowPtr;
        auto scoresPtr = rowPtr + 4;
        auto kps_ptr = rowPtr + 5;
        float score = *scoresPtr;
        
        if (score > PROBABILITY_THRESHOLD) {
            float x = *bboxesPtr++;
            float y = *bboxesPtr++;
            float w = *bboxesPtr++;
            float h = *bboxesPtr;

            float x0 = std::clamp((x - 0.5f * w) * m_ratio, 0.f, m_imgWidth);
            float y0 = std::clamp((y - 0.5f * h) * m_ratio, 0.f, m_imgHeight);
            float x1 = std::clamp((x + 0.5f * w) * m_ratio, 0.f, m_imgWidth);
            float y1 = std::clamp((y + 0.5f * h) * m_ratio, 0.f, m_imgHeight);

            cv::Rect_<float> bbox;
            bbox.x = x0;
            bbox.y = y0;
            bbox.width = x1 - x0;
            bbox.height = y1 - y0;

            std::vector<float> kps;
            for (int k = 0; k < NUM_KPS; k++) {
                float kpsX = *(kps_ptr + 3 * k) * m_ratio;
                float kpsY = *(kps_ptr + 3 * k + 1) * m_ratio;
                float kpsS = *(kps_ptr + 3 * k + 2);
                kpsX = std::clamp(kpsX, 0.f, m_imgWidth);
                kpsY = std::clamp(kpsY, 0.f, m_imgHeight);
                kps.push_back(kpsX);
                kps.push_back(kpsY);
                kps.push_back(kpsS);
            }

            bboxes.push_back(bbox);
            labels.push_back(0);  // All detected objects are people
            scores.push_back(score);
            kpss.push_back(kps);
        }
    }

    nonMaxSuppression(bboxes, scores, labels, PROBABILITY_THRESHOLD, NMS_THRESHOLD, indices);

    std::vector<Object> objects;
    int cnt = 0;
    
    for (auto &chosenIdx : indices) {
        if (cnt >= TOP_K) {
            break;
        }

        Object obj{};
        obj.probability = scores[chosenIdx];
        obj.label = labels[chosenIdx];
        obj.rect = bboxes[chosenIdx];
        obj.kps = kpss[chosenIdx];
        
        // Calcular la distancia estimada - Pasar también los keypoints
        obj.distance = estimateDistance(obj.rect, obj.kps);
        
        objects.push_back(obj);
        cnt += 1;
    }

    return objects;
}

std::vector<Object> YoloV8::postprocessDetect(std::vector<float> &featureVector) {
    // Implementation needed
    return std::vector<Object>();
}

std::vector<Object> YoloV8::postProcessSegmentation(std::vector<std::vector<float>> &featureVectors) {
    // Implementation needed
    return std::vector<Object>();
}

void YoloV8::drawObjectLabels(cv::Mat &image, const std::vector<Object> &objects, unsigned int scale) {
    if (!objects.empty() && !objects[0].boxMask.empty()) {
        cv::Mat mask = image.clone();
        for (const auto &object : objects) {
            int colorIndex = object.label % COLOR_LIST.size();
            cv::Scalar color = cv::Scalar(COLOR_LIST[colorIndex][0] * 255, 
                                        COLOR_LIST[colorIndex][1] * 255, 
                                        COLOR_LIST[colorIndex][2] * 255);
            mask(object.rect).setTo(color, object.boxMask);
        }
        cv::addWeighted(image, 0.5, mask, 0.8, 1, image);
    }

    for (const auto &object : objects) {
        int colorIndex = object.label % COLOR_LIST.size();
        cv::Scalar color = cv::Scalar(COLOR_LIST[colorIndex][0] * 255,
                                    COLOR_LIST[colorIndex][1] * 255,
                                    COLOR_LIST[colorIndex][2] * 255);
        float meanColor = cv::mean(color)[0];
        cv::Scalar txtColor = (meanColor > 0.5) ? cv::Scalar(0, 0, 0) : cv::Scalar(255, 255, 255);

        char text[256];
        sprintf(text, "%s %.1f%% %.1fm", 
                CLASS_NAMES[object.label].c_str(), 
                object.probability * 100,
                object.distance);

        int baseLine = 0;
        cv::Size labelSize = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.35 * scale, scale, &baseLine);
        cv::Scalar txt_bk_color = color * 0.7;

        int x = object.rect.x;
        int y = object.rect.y + 1;

        cv::rectangle(image, object.rect, color, scale + 1);
        cv::rectangle(image, cv::Rect(cv::Point(x, y), 
                     cv::Size(labelSize.width, labelSize.height + baseLine)), 
                     txt_bk_color, -1);
        cv::putText(image, text, cv::Point(x, y + labelSize.height),
                    cv::FONT_HERSHEY_SIMPLEX, 0.35 * scale, txtColor, scale);

        // Dibujar keypoints si están disponibles
        if (!object.kps.empty()) {
            auto &kps = object.kps;
            for (int k = 0; k < NUM_KPS + 2; k++) {
                if (k < NUM_KPS) {
                    int kpsX = std::round(kps[k * 3]);
                    int kpsY = std::round(kps[k * 3 + 1]);
                    float kpsS = kps[k * 3 + 2];
                    if (kpsS > KPS_THRESHOLD) {
                        cv::Scalar kpsColor = cv::Scalar(KPS_COLORS[k][0], KPS_COLORS[k][1], KPS_COLORS[k][2]);
                        cv::circle(image, {kpsX, kpsY}, 5, kpsColor, -1);
                    }
                }
                if (k < SKELETON.size()) {  // Check if k is within SKELETON bounds
                    auto &ske = SKELETON[k];
                    int pos1X = std::round(kps[(ske[0] - 1) * 3]);
                    int pos1Y = std::round(kps[(ske[0] - 1) * 3 + 1]);
                    int pos2X = std::round(kps[(ske[1] - 1) * 3]);
                    int pos2Y = std::round(kps[(ske[1] - 1) * 3 + 1]);
                    float pos1S = kps[(ske[0] - 1) * 3 + 2];
                    float pos2S = kps[(ske[1] - 1) * 3 + 2];

                    if (pos1S > KPS_THRESHOLD && pos2S > KPS_THRESHOLD) {
                        cv::Scalar limbColor = cv::Scalar(LIMB_COLORS[k][0], LIMB_COLORS[k][1], LIMB_COLORS[k][2]);
                        cv::line(image, {pos1X, pos1Y}, {pos2X, pos2Y}, limbColor, 2);
                    }
                }
            }
        }
    }
}
