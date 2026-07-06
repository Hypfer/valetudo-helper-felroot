# valetudo-helper-felboot

Asking people to compile a specifically patched version of sunxi-tools/xfel/etc. would be an exercise in futility.

Therefore:

`valetudo-helper-felboot` is a statically linked single binary tool that contains just enough FEL to do what is needed to root robots.

For usage, simply download the latest binary from the releases section and execute it in a root shell on your linux laptop.<br/>
For usage instructions, see the rooting guides on https://valetudo.cloud

## Credits

It should go without saying that my GPU did this, which makes for better C code than I could ever write.

If you should feel bad about that: Don't.<br/>
GPUs have no soul.

Just like the C-Level of the corps building them.

<br/>
<br/>

This project is just https://github.com/xboot/xfel cut down to what is required to do the thing.<br/>
xfel is licensed under MIT.

Additionally, the project vendors some version of libusb the LLM (likely wrongfully) thought would be the right choice.<br/>
libusb is licensed under the LGPL-2.1.

## Valetudo helpers

Valetudo helpers are a series of small standalone self-contained single-purpose single-file tools built to make
usage and/or installation of Valetudo a bit easier.

As with everything Valetudo, some intermediate computer skills are required. You should know what a network is,
what HTTP is, how a terminal works and that kind of stuff.
Please understand that it's not feasible for this or any open source project to provide basic computer lessons.
