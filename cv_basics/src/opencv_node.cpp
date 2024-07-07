#include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
// #include "sensor_msgs/msg/point_cloud2.hpp"
// #include <image_transport/image_transport.hpp>
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <fstream>

class YOLOV5Model
{
    private:
        const char* path_to_model_;
        const char* path_to_classlist_;
        int frame_count = 0;
        int total_frames = 0;
        float fps = -1;
        std::chrono::high_resolution_clock::time_point start;
        int _started = 0 ;
        std::vector<std::string> class_list;

        std::vector<std::string> className;
        cv::dnn::Net net;
        const float INPUT_WIDTH = 640.0;
        const float INPUT_HEIGHT = 640.0;
        const float SCORE_THRESHOLD = 0.2;
        const float NMS_THRESHOLD = 0.4;
        const float CONFIDENCE_THRESHOLD = 0.4;
        const std::vector<cv::Scalar> colors = {cv::Scalar(255, 255, 0), cv::Scalar(0, 255, 0), cv::Scalar(0, 255, 255), cv::Scalar(255, 0, 0)};

        struct Detection
        {
            int class_id;
            float confidence;
            cv::Rect box;
        };

        cv::Mat format_yolov5(const cv::Mat &source) {
            int col = source.cols;
            int row = source.rows;
            int _max = MAX(col, row);
            cv::Mat result = cv::Mat::zeros(_max, _max, CV_8UC3);
            source.copyTo(result(cv::Rect(0, 0, col, row)));
            return result;
        }
        void load_net(bool is_cuda)
        {
            auto result = cv::dnn::readNet(path_to_model_);
            if (is_cuda)
            {
                std::cout << "Attempty to use CUDA\n";
                result.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
                result.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA_FP16);
            }
            else
            {
                std::cout << "Running on CPU\n";
                result.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
                result.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
            }
            net = result;
        }
    public:
        YOLOV5Model(const char* path_to_model, const char* path_to_classlist)
        {
            path_to_model_ = path_to_model;
            path_to_classlist_ = path_to_classlist;
            class_list = this->load_class_list();
            this->load_net(false);
        }

        std::vector<std::string> load_class_list()
        {
            std::vector<std::string> class_list;
            std::ifstream ifs(path_to_classlist_);
            std::string line;
            while (getline(ifs, line))
            {
                class_list.push_back(line);
            }
            className = class_list;
            return class_list;
        }

        void show(cv::Mat &frame)
        {
            if(_started == 0)
            {
                start = std::chrono::high_resolution_clock::now();
                _started = 1;
            }

            std::vector<Detection> output;
            detect(frame, output);
            int detections = output.size();

            for (int i = 0; i < detections; ++i)
            {

                auto detection = output[i];
                auto box = detection.box;
                auto classId = detection.class_id;
                const auto color = colors[classId % colors.size()];
                cv::rectangle(frame, box, color, 3);
                std::cout << classId << "\n";
                cv::rectangle(frame, cv::Point(box.x, box.y - 20), cv::Point(box.x + box.width, box.y), color, cv::FILLED);
                cv::putText(frame, class_list[classId].c_str(), cv::Point(box.x, box.y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
            }

            if (frame_count >= 30)
            {
                std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
                fps = frame_count * 1000.0 / std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

                frame_count = 0;
                start = std::chrono::high_resolution_clock::now();
            }

            if (fps > 0)
            {

                std::ostringstream fps_label;
                fps_label << std::fixed << std::setprecision(2);
                fps_label << "FPS: " << fps;
                std::string fps_label_str = fps_label.str();

                cv::putText(frame, fps_label_str.c_str(), cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
            }

            cv::imshow("output", frame);

        }

        void detect(cv::Mat &image, std::vector<Detection> &output) 
        {
            cv::Mat blob;

            auto input_image = format_yolov5(image);
            
            cv::dnn::blobFromImage(input_image, blob, 1./255., cv::Size(INPUT_WIDTH, INPUT_HEIGHT), cv::Scalar(), true, false);
            net.setInput(blob);
            std::vector<cv::Mat> outputs;
            net.forward(outputs, net.getUnconnectedOutLayersNames());

            float x_factor = input_image.cols / INPUT_WIDTH;
            float y_factor = input_image.rows / INPUT_HEIGHT;
            
            float *data = (float *)outputs[0].data;

            const int dimensions = 85;
            const int rows = 25200;
            
            std::vector<int> class_ids;
            std::vector<float> confidences;
            std::vector<cv::Rect> boxes;

            for (int i = 0; i < rows; ++i) {

                float confidence = data[4];
                if (confidence >= CONFIDENCE_THRESHOLD) {

                    float * classes_scores = data + 5;
                    cv::Mat scores(1, className.size(), CV_32FC1, classes_scores);
                    cv::Point class_id;
                    double max_class_score;
                    minMaxLoc(scores, 0, &max_class_score, 0, &class_id);
                    if (max_class_score > SCORE_THRESHOLD) {

                        confidences.push_back(confidence);

                        class_ids.push_back(class_id.x);

                        float x = data[0];
                        float y = data[1];
                        float w = data[2];
                        float h = data[3];
                        int left = int((x - 0.5 * w) * x_factor);
                        int top = int((y - 0.5 * h) * y_factor);
                        int width = int(w * x_factor);
                        int height = int(h * y_factor);
                        boxes.push_back(cv::Rect(left, top, width, height));
                    }

                }

                data += 85;

            }

            std::vector<int> nms_result;
            cv::dnn::NMSBoxes(boxes, confidences, SCORE_THRESHOLD, NMS_THRESHOLD, nms_result);
            for (int i = 0; i < nms_result.size(); i++) {
                int idx = nms_result[i];
                Detection result;
                result.class_id = class_ids[idx];
                result.confidence = confidences[idx];
                result.box = boxes[idx];
                output.push_back(result);
            }

            frame_count++;
            total_frames++;
        }
};

class OpenCVNodeSubscriber : public rclcpp::Node
{
  public:
    OpenCVNodeSubscriber()
    : Node("opencv_subscriber")
    {
      // auto qos = rclcpp::QoS( rclcpp::QoSInitialization( RMW_QOS_POLICY_HISTORY_KEEP_LAST, 5 ));
      // qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
   
      subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", 10, std::bind(&OpenCVNodeSubscriber::topic_callback, this, std::placeholders::_1));

      model = new YOLOV5Model("/home/rkroshan/dev_ws/install/cv_basics/lib/cv_basics/config_files/yolov5n.onnx", "/home/rkroshan/dev_ws/install/cv_basics/lib/cv_basics/config_files/classes.txt");
      // publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
      // "cv_image", qos);
    }

  private:
    void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
    { 
      try{
        cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        model->show(frame);
        // cv::imshow("Camera", frame);
        cv::waitKey(1);
      }
      catch (cv_bridge::Exception& e)
      {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      }
      
      // // Convert ROS Image to CV Image
      // cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      // cv::Mat image_raw =  cv_ptr->image;

      // // Image processing
      // cv::Mat cv_image = image_processing(image_raw);

      // // Convert OpenCV Image to ROS Image
      // cv_bridge::CvImage img_bridge = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, cv_image);
      // sensor_msgs::msg::Image out_image; // >> message to be sent
      // img_bridge.toImageMsg(out_image); // from cv_bridge to sensor_msgs::Image

      // // Publish the data
      // publisher_ -> publish(out_image);

    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    // rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    YOLOV5Model* model;
   
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OpenCVNodeSubscriber>());
  rclcpp::shutdown();
  return 0;
}
