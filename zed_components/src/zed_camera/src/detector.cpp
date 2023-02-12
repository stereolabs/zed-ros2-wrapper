#include "detector.h"

YOLODetector::YOLODetector(const std::string &modelPath,
                           const bool &isGPU = true,
                           const cv::Size &inputSize = cv::Size(640, 640))
{
    env = Ort::Env(OrtLoggingLevel::ORT_LOGGING_LEVEL_WARNING, "ONNX_DETECTION");
    sessionOptions = Ort::SessionOptions();

    std::vector<std::string> availableProviders = Ort::GetAvailableProviders();
    auto cudaAvailable = std::find(availableProviders.begin(), availableProviders.end(), "CUDAExecutionProvider");
    OrtCUDAProviderOptions cudaOption;

    if (isGPU && (cudaAvailable == availableProviders.end()))
    {
        std::cout << "GPU is not supported by your ONNXRuntime build. Fallback to CPU." << std::endl;
        std::cout << "Inference device: CPU" << std::endl;
    }
    else if (isGPU && (cudaAvailable != availableProviders.end()))
    {
        std::cout << "Inference device: GPU" << std::endl;
        sessionOptions.AppendExecutionProvider_CUDA(cudaOption);
    }
    else
    {
        std::cout << "Inference device: CPU" << std::endl;
    }

#ifdef _WIN32
    std::wstring w_modelPath = utils::charToWstring(modelPath.c_str());
    session = Ort::Session(env, w_modelPath.c_str(), sessionOptions);
#else
    session = Ort::Session(env, modelPath.c_str(), sessionOptions);
#endif

    Ort::AllocatorWithDefaultOptions allocator;

    Ort::TypeInfo inputTypeInfo = session.GetInputTypeInfo(0);
    std::vector<int64_t> inputTensorShape = inputTypeInfo.GetTensorTypeAndShapeInfo().GetShape();
    this->isDynamicInputShape = false;
    // checking if width and height are dynamic
    if (inputTensorShape[2] == -1 && inputTensorShape[3] == -1)
    {
        std::cout << "Dynamic input shape" << std::endl;
        this->isDynamicInputShape = true;
    }

    for (auto shape : inputTensorShape)
        std::cout << "Input shape: " << shape << std::endl;

    std::cout << session.GetInputNameAllocated(0, allocator).get() << std::endl;
    inputNames.emplace(session.GetInputNameAllocated(0, allocator));
    outputNames.emplace(session.GetOutputNameAllocated(0, allocator));

    // std::cout << "Input name: " << inputNames[0] << std::endl;
    // std::cout << "Output name: " << outputNames[0] << std::endl;

    this->inputImageShape = cv::Size2f(inputSize);
}

void YOLODetector::getBestClassInfo(std::vector<float>::iterator it, const int &numClasses,
                                    float &bestConf, int &bestClassId)
{
    // first 5 element are box and obj confidence
    bestClassId = 5;
    bestConf = 0;

    for (int i = 5; i < numClasses + 5; i++)
    {
        if (it[i] > bestConf)
        {
            bestConf = it[i];
            bestClassId = i - 5;
        }
    }
}

void YOLODetector::preprocessing(cv::Mat &image, float *&blob, std::vector<int64_t> &inputTensorShape)
{
    cv::Mat resizedImage, floatImage;
    cv::cvtColor(image, resizedImage, cv::COLOR_BGR2RGB);
    utils::letterbox(resizedImage, resizedImage, this->inputImageShape,
                     cv::Scalar(114, 114, 114), this->isDynamicInputShape,
                     false, true, 32);

    inputTensorShape[2] = resizedImage.rows;
    inputTensorShape[3] = resizedImage.cols;

    resizedImage.convertTo(floatImage, CV_32FC3, 1 / 255.0);
    blob = new float[floatImage.cols * floatImage.rows * floatImage.channels()];
    cv::Size floatImageSize{floatImage.cols, floatImage.rows};

    // hwc -> chw
    std::vector<cv::Mat> chw(floatImage.channels());
    for (int i = 0; i < floatImage.channels(); ++i)
    {
        chw[i] = cv::Mat(floatImageSize, CV_32FC1, blob + i * floatImageSize.width * floatImageSize.height);
    }
    cv::split(floatImage, chw);
}

std::vector<Detection> YOLODetector::postprocessing(const cv::Size &resizedImageShape,
                                                    const cv::Size &originalImageShape,
                                                    std::vector<Ort::Value> &outputTensors,
                                                    const float &confThreshold, const float &iouThreshold)
{
    std::vector<cv::Rect> boxes;
    std::vector<float> confs;
    std::vector<int> classIds;

    auto *rawOutput = outputTensors[0].GetTensorData<float>();
    std::vector<int64_t> outputShape = outputTensors[0].GetTensorTypeAndShapeInfo().GetShape();
    size_t count = outputTensors[0].GetTensorTypeAndShapeInfo().GetElementCount();
    std::vector<float> output(rawOutput, rawOutput + count);

    // for (const int64_t& shape : outputShape)
    //     std::cout << "Output Shape: " << shape << std::endl;

    // first 5 elements are box[4] and obj confidence
    int numClasses = (int)outputShape[2] - 5;
    int elementsInBatch = (int)(outputShape[1] * outputShape[2]);

    // only for batch size = 1
    for (auto it = output.begin(); it != output.begin() + elementsInBatch; it += outputShape[2])
    {
        float clsConf = it[4];

        if (clsConf > confThreshold)
        {
            int centerX = (int)(it[0]);
            int centerY = (int)(it[1]);
            int width = (int)(it[2]);
            int height = (int)(it[3]);
            int left = centerX - width / 2;
            int top = centerY - height / 2;

            float objConf;
            int classId;
            this->getBestClassInfo(it, numClasses, objConf, classId);

            float confidence = clsConf * objConf;

            boxes.emplace_back(left, top, width, height);
            confs.emplace_back(confidence);
            classIds.emplace_back(classId);
        }
    }

    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confs, confThreshold, iouThreshold, indices);
    // std::cout << "amount of NMS indices: " << indices.size() << std::endl;

    std::vector<Detection> detections;

    for (int idx : indices)
    {
        Detection det;
        det.box = cv::Rect(boxes[idx]);
        utils::scaleCoords(resizedImageShape, det.box, originalImageShape);

        det.conf = confs[idx];
        det.classId = classIds[idx];
        detections.emplace_back(det);
    }

    return detections;
}

std::vector<Detection> YOLODetector::detect(cv::Mat &image, const float &confThreshold = 0.4,
                                            const float &iouThreshold = 0.45)
{
    float *blob = nullptr;
    std::vector<int64_t> inputTensorShape{1, 3, -1, -1};
    this->preprocessing(image, blob, inputTensorShape);

    size_t inputTensorSize = utils::vectorProduct(inputTensorShape);

    std::vector<float> inputTensorValues(blob, blob + inputTensorSize);

    std::vector<Ort::Value> inputTensors;

    Ort::MemoryInfo memoryInfo = Ort::MemoryInfo::CreateCpu(
        OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);

    inputTensors.push_back(Ort::Value::CreateTensor<float>(
        memoryInfo, inputTensorValues.data(), inputTensorSize,
        inputTensorShape.data(), inputTensorShape.size()));

    const char *inputNames_[] = {inputNames->get()};
    const char *outputNames_[] = {outputNames->get()};

    std::vector<Ort::Value>
        outputTensors = this->session.Run(Ort::RunOptions{nullptr},
                                          inputNames_,
                                          inputTensors.data(),
                                          1,
                                          outputNames_,
                                          1);

    cv::Size resizedShape = cv::Size((int)inputTensorShape[3], (int)inputTensorShape[2]);
    std::vector<Detection> result = this->postprocessing(resizedShape,
                                                         image.size(),
                                                         outputTensors,
                                                         confThreshold, iouThreshold);

    delete[] blob;

    return result;
}
