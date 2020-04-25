#include "usb_names.h"

#define MIDI_NAME {'E','2','5','6'}
#define MIDI_NAME_LEN  4

// Do not change this part.  This exact format is required by USB.

struct usb_string_descriptor_struct usb_string_product_name = {
        2 + MIDI_NAME_LEN * 2,
        3,
        MIDI_NAME
};
