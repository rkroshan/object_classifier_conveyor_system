#include <fstream>

#include <opencv2/opencv.hpp>



int main(int argc, char **argv)
{
    cv::Mat frame;
    cv::VideoCapture capture("config_files/sample.mp4");
    if (!capture.isOpened())
    {
        std::cerr << "Error opening video file\n";
        return -1;
    }

    YOLOV5Model model("config_files/yolov5n.onnx", "config_files/classes.txt");
    
    while (true)
    {
        capture.read(frame);
        if (frame.empty())
        {
            std::cout << "End of stream\n";
            break;
        }

        model.show(frame);

        if (cv::waitKey(1) != -1)
        {
            capture.release();
            std::cout << "finished by user\n";
            break;
        }
    }

    return 0;
}