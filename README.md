# GuitarFx

Make guitar/bass effects with a
[Teensy](https://www.pjrc.com/store/teensy40.html) and a
[Pmod2](https://digilent.com/reference/pmod/pmodi2s2/start)!

# The Big Idea

The biggest challenge in music is to have an _unique_ sound that defines *you*.
However, your tone depends on the gear you buy, and you can only get that tone
by mixing and matching with whatever is available. What if you had a blank
canvas to design your sound? What would you do with the possibility? What if
the hardware and software was open-source?

# The Solution

GuitarFx is a project that is currently developing a single instrument pedal
kit based on [Arduino](https://www.arduino.cc/). It currently requires
[Teensy](https://www.pjrc.com/store/teensy40.html) and a
[Pmod2](https://digilent.com/reference/pmod/pmodi2s2/start) to use with Teensy
audio library. There is a a framework written on top of it to create your own
sound. Currently, there are three parameter knobs supported, denoted `A`, `B`,
`C`.

# Build setup
1. Download a copy of Arduino IDE 1.x (2.x is still not supported for this
   project) from the [Arduino Website](https://www.arduino.cc/en/software)
1. Get Teensyduino addon from [PJRC's Website](https://www.pjrc.com/teensy/td_download.html)
1. Open `GuitarFx/GuitarFx.ino` on Arduino IDE, and click verify and upload

# Example effects

## Distortion

This is a simple distortion effect for guitar and bass. `A` knob controls the
gain of the distortion pre-amp. To get a simple crunchy-tone, dial the `A` knob
at 12:00, and dial it to 5:00 to get a solo lead. `B` knob controls the tone of
the effect, as min position makes distortion sound dull while max position
makes it sound bright. `C` knob controls the blend between clas-A style
distortion, and class-B style distortion. Class-A style distortion are similar
to Boss DS-1, MXR Distortion+, and clas-B style distortions emulate the sound
of tubes.

# TODO

1. `[DONE]` Create prototype
1. `[DONE]` Write framework
1. `[DONE]` Create an example effect
1. Provide more example effects
1. Make schematic and PCB layout
1. Add 4th parameter knob. 4 seems to be a good number

# License

The code is under GPLv3. See [LICENSE](LICENSE) for more info.

# Sponsor!

This project is done on my free time and using my own resource. If you like
this project and want to help out and contribute, I would really appreciate it.
