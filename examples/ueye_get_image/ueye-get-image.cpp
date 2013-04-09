#include <iostream>
#include <laustracker/laustracker.h>
#include <ueyecam.h>

using namespace std;

int main(int argc, char *argv[])
{
    UEyeCam ucam;
    ucam.init((HIDS) 0);

    Mat image;

    while (waitKey(10) != 27)
    {
        if (ucam.getImg(image) == UEYE_SUCCESS)
        {
            laustracker_imshow("test", image);
        }
    }
    return 0;
}

