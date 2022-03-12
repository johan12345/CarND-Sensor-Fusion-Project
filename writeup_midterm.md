# Writeup for midterm project (Task ID_S1_EX2)

Here are some examples of vehicles with varying degrees of visibility in the point-cloud: 

1. Vehicle behind the Waymo car -> well visible from the front\
  ![car1](img/car1.png)
2. Trailer whose loading platform is partly occluded by its rear end\
   ![car2](img/car2.png)
3. Car right next to the Waymo car -> partly within the blind spot of the top Lidar\
   ![car3](img/car3.png)
4. Car in front of and far away the Waymo car -> rear bumper detected at low resolution\
   ![car4](img/car4.png)
5. Vehicles seen from the side and partly occluding each other\
   ![car5](img/car5.png)
6. Cars in front of the Waymo car -> rear bumper visible\
   ![car6](img/car6.png)

For most vehicles, the front or rear bumpers are relatively stable features that appear as
roughly parallel curved lines. Around the head/tail lights these lines bend a bit differently to follow
the shape of the lights. On the other hand, e.g. the windows do not appear in the Lidar detections
as they are transparent to the wavelengths of light that the Lidar is using.

Looking at an excerpt from the range image (range channel on top, intensity channel on the bottom), we
can also see that the abovementioned features reflect the Lidar beam very well (with high intensity):
![range image](img/range_image.png)