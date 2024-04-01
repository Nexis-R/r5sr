#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "../quirc-lib/quirc.h"

#include <foxglove_msgs/msg/image_annotations.hpp>
#include <foxglove_msgs/msg/text_annotation.hpp>
#include <foxglove_msgs/msg/points_annotation.hpp>
#include <foxglove_msgs/msg/point2.hpp>

using std::placeholders::_1;
using namespace std;

class qrcv : public rclcpp::Node{
    public:
    qrcv(): Node("qr_processer"){
        publisher_ = this->create_publisher<foxglove_msgs::msg::ImageAnnotations>("qr_text", 10);
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "image_raw", 10, std::bind(&qrcv::topic_callback, this, _1)
        );
        
    }
    private:
    mutable int miss=0;
    struct QRCodeData {
        foxglove_msgs::msg::Point2 corners[4]; // 座標を格納するvector
        std::string text; // QRコードのデータ
        bool result;// qrが読み取れたらtrue 読み取れなかったらfalse
    };
    QRCodeData qr_detector(cv::Mat image_mat)const{
        QRCodeData result;
        result.result=false;

        // 画像の幅と高さを取得
        int width = image_mat.cols;
        int height = image_mat.rows;

        // Quircオブジェクトの作成
        struct quirc *qr = quirc_new();
        quirc_resize(qr, width, height);

        // グレースケールのMatオブジェクトを作成
        cv::Mat grey_mat;
        cv::cvtColor(image_mat, grey_mat, cv::COLOR_BGR2GRAY);

        // グレースケール画像を8bit符号なし整数型に変換
        cv::Mat grey_uchar_mat;
        grey_mat.convertTo(grey_uchar_mat, CV_8U);

        // QRコードを検出
        uint8_t *qr_image = quirc_begin(qr, NULL, NULL);
        memcpy(qr_image, grey_uchar_mat.data, width * height);
        quirc_end(qr);

        int num_codes = quirc_count(qr);
        std::string detect_data; // QRコードの内容を格納する変数

        for (int i = 0; i < num_codes; i++) {
            struct quirc_code code;
            quirc_extract(qr, i, &code);
            for (int j = 0; j < 4; j++) {
                result.corners[j].x = code.corners[j].x;
                result.corners[j].y = code.corners[j].y;
            }

            struct quirc_data data;
            quirc_decode_error_t err = quirc_decode(&code, &data);
            if (!err) {
                // QRコードの内容をdate変数に格納
                result.text = std::string(reinterpret_cast<char*>(data.payload), data.payload_len);
                result.result=true;
                break;//
            }
        }
        
        quirc_destroy(qr);
        
        return result;
    }
    foxglove_msgs::msg::ImageAnnotations fox_covert(QRCodeData data)const{
        foxglove_msgs::msg::ImageAnnotations msg;
        foxglove_msgs::msg::TextAnnotation text_annotation;
        builtin_interfaces::msg::Time timestamp = this->now();
        text_annotation.timestamp.sec = timestamp.sec; // Fill in appropriate timestamp
        text_annotation.timestamp.nanosec = timestamp.nanosec;
        text_annotation.text = data.text; // Set the received string here
        text_annotation.position.x = 10; // Fill in appropriate position
        text_annotation.position.y = 25;        text_annotation.text_color.r = 0; // Set appropriate text color
        text_annotation.text_color.g = 0;
        text_annotation.text_color.b = 0;
        text_annotation.text_color.a = 1.0;
        text_annotation.background_color.r = 255; // Set appropriate background color
        text_annotation.background_color.g = 255;
        text_annotation.background_color.b = 255;
        text_annotation.background_color.a = 1.0;

        if(data.text.size()>=20){
            text_annotation.font_size = 20.0; // Set appropriate font size
        }else if(data.text.size()>=30){
            text_annotation.font_size = 10.0; // Set appropriate font size
        }else{
            text_annotation.font_size = 50.0; // Set appropriate font size
        }
        
        
        msg.texts.push_back(text_annotation);

        if(!data.result){return msg;}//not detectedのとき除外
        
        foxglove_msgs::msg::PointsAnnotation point_annotation;
        point_annotation.timestamp.sec = timestamp.sec; 
        point_annotation.timestamp.nanosec = timestamp.nanosec;
        point_annotation.type=2;
        point_annotation.outline_color.r=0;
        point_annotation.outline_color.g=0;
        point_annotation.outline_color.b=255;
        point_annotation.outline_color.a=1.0;
        point_annotation.thickness=5.0;

        for(int i=0;i<4;i++){
            point_annotation.points.push_back(data.corners[i]);
        }

        msg.points.push_back(point_annotation);

        return msg;
        
    }
    void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) const{
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
        
        cv::Mat cv_img = cv_ptr->image;

        QRCodeData detect_data = qrcv::qr_detector(cv_img);
        if(!detect_data.result){
            detect_data.text="not detector";
        }
        
        foxglove_msgs::msg::ImageAnnotations img_ann = fox_covert(detect_data);
        //読み込みがちらつくので読み取りを複数回に一回出力にする
        if(detect_data.result||qrcv::miss==4){
            publisher_->publish(img_ann);
            qrcv::miss=0;
        }else{
            qrcv::miss++;
        }
    }
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<foxglove_msgs::msg::ImageAnnotations>::SharedPtr publisher_;
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<qrcv>());
    rclcpp::shutdown();
    return 0;
}