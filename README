

This is a collection of code example using a stm32 dma driver that i wrote.

Each different use of the dma driver is in a different branch of this repo.

The driver can be found in a collection of utilities that are gathered here :
git@github.com:alex31/chibios_enac_various_common.git

the driver is on only two files : hal_stm32_dma.h and hal_stm32_dma.c

1) Goals 
I have initially wrote this driver for my lectures, to let dma be
easier to manage for my students. When they address dma, they are
already accustomed to the chibios driver API : most of the complexity
and hardware dependant stuff is in a configuration structure, and the
functions are few, and simple to use.

2) Pro
Mimic other ChibiOS drivers, offer both asynchronous and synchronous
api (like the ADC driver), manage timeout, both in one shot
(synchronous) or continuous (asynchronous) mode.

3) Cons
tested on dmav1.1, dmav2, should work on dmav1 (not tested),
but do not address new dma : bdma, dmav3, mdma, dmamux. Will have to
eventually add theses beasts, and it will complicate the structure : 2
files will no more be enough, and i will have to follow the classic
hardware independent + low level + .mk structure.

nearly all the DMA registers are mirrored in the configuration
structure, but the mem1p field. At first, like in ChibiOS, i was
thinking that in any case half buffer scheme is sufficient, but, after
thinking about, implementing double buffer operations would be
interesting : one can use memory pool or linked list, without spurious
copies, instead of double buffer based operations.

4) Todo Address
all the points of the Cons :-)





