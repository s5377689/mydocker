#include <opencv2/opencv.hpp>
#include <string>
#include <cmath>
#include <vector>
#include <filesystem>

using namespace std;
using namespace cv;
namespace fs = std::filesystem;

// Image scale to show
double displayScale = 1.0;
const double SCALE_STEP = 0.1;
const double MIN_SCALE = 0.1;
const double MAX_SCALE = 2.0;

int zoomSlider = 5; // initial value
int maxZoomSlider = 20;

void onZoomTrackbar(int value, void*) {
    displayScale = value / 10.0;
}

vector<vector<Point>> allPolygonPoints;
size_t currentImgIndex = 0;
vector<bool> selectionClosed;

void onMouse(
    int event, int display_x, int display_y,
    int flags, void*)
{
    if (event == cv::EVENT_MOUSEWHEEL)
    {
        int delta = cv::getMouseWheelDelta(flags);
        if (delta > 0)
        {
            displayScale = std::min(displayScale + SCALE_STEP, MAX_SCALE);
        }
        else
        {
            displayScale = std::max(displayScale - SCALE_STEP, MIN_SCALE);
        }
        cout << "Zoom scale: " << displayScale << endl;
        return;
    }

    // Adjust the coordinates based on the display scale
    int x = static_cast<int>(display_x / displayScale);
    int y = static_cast<int>(display_y / displayScale);

    vector<Point> & polygonPoints = allPolygonPoints[currentImgIndex];

    if (event == cv::EVENT_LBUTTONDOWN)
    {
        // Add the first point to the polygon
        if (polygonPoints.empty())
        {
            polygonPoints.emplace_back(x, y);
            return;
        }

        float distance = cv::norm(Point(x, y) - polygonPoints[0]);

        if (distance < 12.0 && polygonPoints.size() >= 3)
        {
            // Close the polygon
            selectionClosed[currentImgIndex] = true;
        }
        else if (!selectionClosed[currentImgIndex])
            polygonPoints.emplace_back(x, y);
    }
    else if (event == cv::EVENT_RBUTTONDOWN)
    {
        if (!polygonPoints.empty())
        {
            polygonPoints.pop_back();
            selectionClosed[currentImgIndex] = false;
        }
    }
}

void drawPolygon(
    Mat & img,
    const size_t img_index)
{
    const auto & polygonPoints = allPolygonPoints[img_index];

    for (size_t i = 0; i < polygonPoints.size(); ++i)
    {
        circle(img, polygonPoints[i], 3, Scalar(0, 255, 0), -1);
        if (i > 0)
            line(img, polygonPoints[i-1], polygonPoints[i], Scalar(255, 0, 0), 2);
    }
    if (selectionClosed[img_index])
    {
        line(img, polygonPoints.back(), polygonPoints[0], Scalar(255, 0, 0), 2);
    }
}

void cropAndSave(
    const Mat & img,
    const vector<Point> & polygonPoints,
    const fs::path & output_path,
    const fs::path & output_filename)
{
    // Create a mask for the polygon
    Mat mask = Mat::zeros(img.size(), CV_8UC1);
    vector<vector<Point>> fillContAll = { polygonPoints };
    fillPoly(mask, fillContAll, Scalar(255));

    Mat result;
    img.copyTo(result, mask);

    // Crop the bounding rectangle of the polygon
    Rect bbox = boundingRect(polygonPoints);
    Mat cropped = result(bbox);

    // Save the cropped image
    string output_full_path = (output_path / output_filename.filename()).string();
    imwrite(output_full_path, cropped);
    cout << "Saved cropped image to: " << output_full_path << endl;
}


int main(int argc, char* argv[])
{
    if (argc < 2)
    {
        cout << "Usage: crop <folder_path>" << endl;
        return -1;
    }

    fs::path folder_path(argv[1]);
    if (!fs::exists(folder_path) || !fs::is_directory(folder_path))
    {
        cout << "Error: The specified path does not exist or is not a directory." << endl;
        return -1;
    }

    // Collect all image files in the directory
    vector<fs::path> image_files;
    for (const auto & entry : fs::directory_iterator(folder_path))
    {
        string ext = entry.path().extension().string();
        if (ext == ".jpg" || ext == ".png" || ext == ".jpeg" || ext == ".bmp")
        {
            image_files.push_back(entry.path());
        }
    }

    if (image_files.empty())
    {
        cout << "No image files found in the specified folder." << endl;
        return -1;
    }

    std::sort(image_files.begin(), image_files.end());
    allPolygonPoints.resize(image_files.size());
    selectionClosed.resize(image_files.size(), false);

    // Create the ouptut folder
    fs::path output_folder =
        folder_path.parent_path() / (folder_path.filename().string() + "_cropped");
    fs::create_directories(output_folder);

    size_t index = 0;

    namedWindow("Select ROI");
    cv::createTrackbar("Zoom", "Select ROI", &zoomSlider, maxZoomSlider, onZoomTrackbar);
    setMouseCallback("Select ROI", onMouse);

    // Image loading loop
    while (true)
    {
        Mat img = imread(image_files[currentImgIndex].string());
        if (img.empty())
        {
            cout << "Warning: Failed to load " << image_files[index] << endl;
            continue;
        }

        // Polygon points viewing and selection loop
        while (true)
        {
            Mat temp = img.clone();
            std::stringstream ss;
            ss << "[a]: <-, [d]: ->, [Enter]: save (" << currentImgIndex + 1 << "/" << image_files.size() << ")";
            drawPolygon(temp, currentImgIndex);

            // Resize the image for display
            Mat display;
            resize(temp, display, Size(), displayScale, displayScale);

            putText(
                temp,
                ss.str(),
                Point(5, 12), 
                FONT_HERSHEY_COMPLEX, 0.45,
                Scalar(100, 100, 100), 1.5);

            imshow("Select ROI", display);

            int key = waitKey(20);
            if (key == 27) // Escape key
            {
                destroyAllWindows();
                return 0;
            }
            else if (key == 'd')
            {
                currentImgIndex = (currentImgIndex + 1) % image_files.size();
                break;
            }
            else if (key == 'a')
            {
                currentImgIndex = (currentImgIndex == 0) ? image_files.size() - 1 : currentImgIndex - 1;
                break;
            }
            else if (key == 13 || key == 10) // Enter key to finalize and export
            {
                for (size_t i = 0; i < image_files.size(); ++i)
                {

                    if (!selectionClosed[i])
                        continue;

                    Mat img_i = imread(image_files[i].string());
                    if (!img_i.empty())
                        cropAndSave(img_i, allPolygonPoints[i], output_folder, image_files[i]);
                }
                destroyAllWindows();
                return 0;
            }
        }
    }

    return 0;
}