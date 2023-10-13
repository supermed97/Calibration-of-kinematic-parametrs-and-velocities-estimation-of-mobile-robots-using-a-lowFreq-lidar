#include <opencv2/opencv.hpp>

using namespace cv;
void onMouse(int event, int x, int y, int flags, void* userdata) {
    if (event == EVENT_MOUSEMOVE) {
        Mat img = *((Mat*)userdata);
        int width = img.cols;
        int height = img.rows;

        // Check if mouse is inside image bounds
        if (x >= 0 && x < width && y >= 0 && y < height) {
            // Convert mouse coordinates to image coordinates
            int imgX = x;
            int imgY = height - y - 1;

            // Print coordinates to console
            std::cout << "Cursor position: (" << imgX << ", " << imgY << ")" << std::endl;
        }
    }
}


int main() {
    // Load image
   Mat img0 = imread("/home/mohammed/Downloads/diff_drive.jpg");
     
    Mat img;

    cv::resize(img0, img, cv::Size(640, 480)); 
    // Create window
    namedWindow("2D Visualizer", WINDOW_NORMAL);

    // Set initial position and angle
    double x = 50.0;
    double y = 50.0;
    double theta = 0.0;

    // Set velocity and steering angle
    double v = 0.0;  // velocity in pixels per second
    double phi = 0.9;  // steering angle in degrees

    // Define grid parameters
    const int grid_size = 50;
    const int num_cols = img.cols / grid_size;
    const int num_rows = img.rows / grid_size;

    // Main loop
    while (true) {
        // Clear image
        img.setTo(Scalar(255, 255, 255));  // white background

        // Draw grid
        for (int i = 0; i < num_cols; i++) {
            for (int j = 0; j < num_rows; j++) {
                int x1 = i * grid_size;
                int y1 = j * grid_size;
                int x2 = (i + 1) * grid_size;
                int y2 = (j + 1) * grid_size;
                rectangle(img, Point(x1, y1), Point(x2, y2), Scalar(0, 0, 0), 1);
            }
        }

        // Update position and angle based on velocity and steering angle
        double radians = theta * M_PI / 180.0;  // Convert degrees to radians
        x += v * cos(radians);
        y += v * sin(radians);
        theta += phi;

        // Draw current position and angle on image
        int ix = static_cast<int>(x);
        int iy = static_cast<int>(y);
        circle(img, Point(ix, iy), 5, Scalar(0, 0, 255), -1);  // red circle
        int dx = static_cast<int>(20 * cos(radians));
        int dy = static_cast<int>(20 * sin(radians));
        line(img, Point(ix, iy), Point(ix + dx, iy + dy), Scalar(0, 255, 0), 2);  // green line

	// Display cursor coordinates on window
        std::ostringstream ss;
        ss << "x: " << ix << " y: " << iy;
        putText(img, ss.str(), Point(10, img.rows - 10), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);

        // Display image
        imshow("2D Visualizer", img);

	setMouseCallback("2D Visualizer", onMouse, &img);

        // Wait for a key press or exit if 'q' is pressed
        char key = waitKey(10);
        if (key == 'q' || key == 'Q' || key == 27) {
            break;
        }
    }

    return 0;
}

