# GuitarFx

Make guitar/bass effects with a
[Teensy](https://www.pjrc.com/store/teensy40.html) and a
[Pmod2](https://digilent.com/reference/pmod/pmodi2s2/start)!

# The big idea

The biggest challenge in music is to have an unique sound that defines
yourself. However, your tone depends on the gears you buy, and you can only get
that tone by mixing and matching with whatever is available. What if you had a
blank canvas to design your sound? What would you do with the possibility? What
if the hardware and software was open-source?

# What this is

GuitarFx is a project that is currently developing a single instrument pedal
kit based on [Arduino](https://www.arduino.cc/). It currently requires
[Teensy](https://www.pjrc.com/store/teensy40.html) and a
[Pmod2](https://digilent.com/reference/pmod/pmodi2s2/start) to use with Teensy
audio library. There is a a framework written on top of it to create your own
sound. Currently, there are three parameter knobs supported, denoted `A`, `B`,
`C`.

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

github: ashafq
