#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include <string>


image_transport::CameraSubscriber image_sub;
image_transport::CameraPublisher image_pub;

//Parameters fo rescaled image
uint target_width;
uint target_height;
std::string source_topic;
std::string target_topic;
std::string target_encoding;

bool loadParamters(ros::NodeHandle n);
void printParameters();
bool sanityCheck(const sensor_msgs::ImageConstPtr& imgSource);
void imageCallback(const sensor_msgs::ImageConstPtr& imgSource,
                   const sensor_msgs::CameraInfoConstPtr &infoSource);



bool loadParamters(ros::NodeHandle n)
{
    int target_width_int = 0;
    int target_height_int = 0;

    if(!n.getParam("/image_resampler/target_width", target_width_int))
    {
        ROS_ERROR("Missing parameter: /image_resampler/target_width");
        return false;
    }
    if(!n.getParam("/image_resampler/target_height", target_height_int))
    {
        ROS_ERROR("Missing parameter: /image_resampler/target_height");
        return false;
    }
    if(!n.getParam("/image_resampler/source_topic", source_topic))
    {
        ROS_ERROR("Missing parameter: /image_resampler/source_topic");
        return false;
    }
    if(!n.getParam("/image_resampler/target_topic", target_topic))
    {
        ROS_ERROR("Missing parameter: /image_resampler/target_topic");
        return false;
    }
    if(!n.getParam("/image_resampler/target_encoding", target_encoding))
    {
        ROS_ERROR("Missing parameter: /image_resampler/target_encoding");
        return false;
    }

    target_width = (uint)target_width_int;
    target_height = (uint)target_height_int;

    return true;
}

void printParameters()
{
    ROS_INFO("target_width: %u", target_width);
    ROS_INFO("target_height: %u", target_height);
    ROS_INFO("source_topic: %s", source_topic.c_str());
    ROS_INFO("target_topic: %s", target_topic.c_str());
    ROS_INFO("target_encoding: %s", target_encoding.c_str());
}


bool sanityCheck(const sensor_msgs::ImageConstPtr& imgSource)
{
    //No bigEndian encoded images
    if(imgSource->is_bigendian)
    {
        ROS_ERROR("BigEndian Source not supported");
        return false;
    }

    //No upscaling
    if(target_width > imgSource->width || target_height > imgSource->height)
    {
        ROS_ERROR("Upscaling not supported - Source: %dx%d - Target %dx%d",
                  imgSource->width, imgSource->height, target_width, target_height);
        return false;
    }

    //Only supported encodings
    if(imgSource->encoding != "16UC1")
    {
        ROS_ERROR("Only 16UC1 for source encoding supported");
        return false;
    }
    if(target_encoding != "32FC1")
    {
        ROS_ERROR("Only 32FC1 for target encoding supported");
        return false;
    }

    return true;
}


void imageCallback(const sensor_msgs::ImageConstPtr& imgSource,
                   const sensor_msgs::CameraInfoConstPtr &infoSource)
{
    //Unfortunately we need to do this every time
    //because form of input topic could change at any time
    if(!sanityCheck(imgSource))
    {
        ros::shutdown();
        return;
    }

    static sensor_msgs::Image imgResampled;
    static sensor_msgs::CameraInfo infoResampled;

    //Starting positions to copy from
    uint xOffset = 0;
    uint yOffset = 0;

    //How much bigger is the source image?
    float xScalar = (float)imgSource->width / (float)target_width;
    float yScalar = (float)imgSource->height / (float)target_height;
    float scalar = 0.0f;

    //Set image metadata for the rescaled image
    imgResampled.header = imgSource->header;
    imgResampled.is_bigendian = imgSource->is_bigendian;
    imgResampled.encoding = target_encoding;

    //How many data[] elemts make up one pixel?
    //As long as encoding is equal this is the same for source and target image
    uint elemtsPerTargetPixel = sensor_msgs::image_encodings::bitDepth(imgResampled.encoding) / 8;

    imgResampled.width = target_width;
    imgResampled.height = target_height;
    imgResampled.step = target_width * elemtsPerTargetPixel;

    //Rescale data array to match new resolution
    imgResampled.data.resize(imgResampled.step * target_height);

    //One pixel is comprised of several (elemtsPer***Pixel, depends on encoding) adjanct elements
    //in the data array of the image. We interpret several of these elements as one pointer adress.
    //ATTENTION: Only works for smallEndian encoded images

    //Example:
    //0101 1111 (data<4bit>[0] and data<4bit>[1])   <=>   10101111 (pointer<8bit>[0])

    //Provide pointers for different encodings
    const short* source = reinterpret_cast<const short*>(&imgSource->data[0]);
    float* target = reinterpret_cast<float*>(&imgResampled.data[0]);

    //We need to determin wether the rescaled imaged needs to be cropped horizontally or vertically
    //This depends on the aspect ratio of input and output images
    //Offset values are the point where, in the input image, we start to copy from
    if(xScalar > yScalar)
    {
        scalar = yScalar;
        xOffset = (imgSource->width - (target_width * yScalar)) / 2;
        yOffset = 0;
    } else if(xScalar < yScalar)
    {
        scalar = xScalar;
        xOffset = 0;
        yOffset = (imgSource->height - (target_height * xScalar)) / 2;
    } else //Same aspect ratio
    {
        scalar = yScalar;
        yOffset = 0;
        xOffset = 0;
    }

    //Copy cameraInfo, adapt to the new resolution
    infoResampled = *infoSource;
    infoResampled.width = target_width;
    infoResampled.height = target_height;
    infoResampled.K[0] = infoSource->K[0] / scalar;
    infoResampled.K[2] = infoSource->K[2] / scalar;
    infoResampled.K[4] = infoSource->K[4] / scalar;
    infoResampled.K[5] = infoSource->K[5] / scalar;

    //Copy pixels from source image to rescaled image (target), convert to float
    //All "coordinates" / "positions" are in pixels
    for(uint row = 0; row < target_height; row++)
    {
        uint source_row = (float)row * (float)scalar + yOffset;
        uint source_row_start = imgSource->width * source_row;
        uint target_row_start = target_width * row;

        for(uint col = 0; col < target_width; col++)
        {
            uint source_col = (float)col * float(scalar);
            uint source_index = source_row_start + source_col + xOffset;
            uint target_index = target_row_start + col;

            //Convert mils (as short) to metres (as float)
            target[target_index] = (float)source[source_index] / 1000;
        }
    }

//    ROS_INFO("%d x %d (%s) --> %d x %d (%s)",
//             imgSource->width,
//             imgSource->height,
//             imgSource->encoding.c_str(),
//             imgResampled.width,
//             imgResampled.height,
//             imgResampled.encoding.c_str());

    //Output modified video topic
    image_pub.publish(imgResampled, infoResampled);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_resampler");

    ros::NodeHandle n;

    image_transport::ImageTransport it(n);

    //Load parameters from image_resampler_config/settings.yaml
    if(!loadParamters(n))
    {
        ROS_ERROR("Parameter loading failed, exiting!");
        ros::shutdown();
        return 1;
    }

    image_sub = it.subscribeCamera(source_topic, 1, imageCallback);
    image_pub = it.advertiseCamera(target_topic, 1);

    ros::spin();

    return 0;
}
