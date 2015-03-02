# Abstract #
Automatic wharehouses are becoming more and more common nowadays. In particular, big e-commerce companies such as Amazon would have a good advantage in completely automate the delivery of objects from the wharehouse to their customers.

One of the tasks which is still not fully automated and thus is still human-made is picking bought items from the shelves of the wharehouse and putting them into their package or container to be sent away in an automated way.

The Amazon Picking Challenge ( http://www.amazonpickingchallenge.org ) is a challenge from the Amazon company which will be held from 2015-05-26 to 2015-05-28 at ICRA (International Conference on Robotics and Automation); its main goal is to build a fully automated system for solving the problem stated above into a simplified environment.

More in detail, contestants' robots will be placed in front of a stationary Kiva shelf. The robot will have to *independently* move the items from the shelf into a container in the robot workcell. Details on the items to be managed and on the contents of the bin will be given at contest time as a single JSON file.

Score to the contest is given with a cumulative point system, awarding robots for each item succesfully moved and giving penalties for each broken or damaged item.

The Isaac Team robotic project will participate to the contest and study alternative methods of performing the task without putting the focus on computer vision-guided gripping system, which would probably break or damage soft or openable objects such as books or journals.
