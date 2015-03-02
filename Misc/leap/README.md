# Leap motion depth frame capture
This is done by subclassing *Leap::Listener*.

Every leap application included into the default sdk package is written in Java (think - looking at the API), so the Listener concept is quite clear.

Every *on\** method has a *Leap::Controller* as a parameter, giving it a context.

Interesting methods to override:
  - *void onFrame*: callback for a new frame. See the relative code:

    const Leap::Frame frame = controller.frame();
    Leap::ImageList images = frame.images();

    Leap::Image image = images[0];
    /* Now image.height(), image.width(), image.data() are available for processing. */
