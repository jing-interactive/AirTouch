#include "cvButtons.h"

using namespace cv;

void cvButtonsOnMouse(int e, int x, int y, int f, void* param)
{
    CvButtons* btn = (CvButtons*)param;
    btn->setMouseState(e,x,y,f);
}

void CvButtons::paintButtons(cv::Mat& img)
{
    const Scalar C1 = Scalar::all(255);
    const Scalar C2 = Scalar::all(0);

    for (std::vector<PushButton>::iterator it = buttonList.begin();
        it != buttonList.end();
        ++ it)
    {
        // Grab button variables:
        int x = it->x_pos;
        int y = it->y_pos;
        int w = it->width;
        int h = it->height;
        int x2 = x+w;
        int y2 = y+h;

        // Highlight mouse over position:
        if (mx >= x && mx <= x2 && my >= y && my <= y2)
        {
            rectangle(img, Point(x-4,y-4), Point(x2+4,y2+4), C2, 1,LINE_AA);

            // Check for mouse pressed event:
            if (me == EVENT_LBUTTONDOWN/* || mf & EVENT_FLAG_LBUTTON */)
            {
                // Check if toggleRef button has to change state:
                if (it->toggleRef != NULL) *it->toggleRef = !*it->toggleRef;

                // Call callback function:
                if (it->cb != NULL)
                {
                    it->cb(it->toggleRef ? *it->toggleRef : 0);
                }

                // Draw confirmation rectangle:
                rectangle(img, Point(x,y), Point(x2,y2), C2, FILLED, LINE_AA);

                // Reset event (avoid flickering buttons):
                me = EVENT_MOUSEMOVE;
            }
        }

        // Draw toggleRef state:
        if (it->toggleRef && *it->toggleRef == 1)
            rectangle(img, Point(x,y), Point(x2,y2), C2, FILLED, LINE_AA);

        // Draw button with text:
        rectangle(img, Point(x,y), Point(x2,y2), C2,1,LINE_AA);
        if (it->toggleRef && *it->toggleRef == 1)
            putText(img, it->text, Point(x+5,y+15), FONT_HERSHEY_PLAIN, 1.0f, C1);
        else
            putText(img, it->text, Point(x+5,y+15), FONT_HERSHEY_PLAIN, 1.0f, C2);
    }
}

