package org.robotroopers.wrapper

class NativeLib {

    /**
     * A native method that is implemented by the 'wrapper' native library,
     * which is packaged with this application.
     */
    external fun stringFromJNI(): String

    companion object {
        // Used to load the 'wrapper' library on application startup.
        init {
            System.loadLibrary("wrapper")
        }
    }
}