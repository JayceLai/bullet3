if [ $# -ne 0 ]; then
    ndk_path=$1
elif [ -n "$ANDROID_NDK_HOME" ]; then
    ndk_path=$ANDROID_NDK_HOME
elif [ -n "$ANDROID_NDK_ROOT" ]; then
    ndk_path=$ANDROID_NDK_ROOT
elif [ -n "$NDK_ROOT" ]; then
    ndk_path=$NDK_ROOT
else
    echo "Cannot find NDK root path"
    exit 1
fi

cmake -Bbuild-for-vscode -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_TOOLCHAIN_FILE="$ndk_path/build/cmake/android.toolchain.cmake" -DCMAKE_MAKE_PROGRAM="$ndk_path/prebuilt/windows-x86_64/bin/make" -DANDROID_PLATFORM=android-21 -G"Unix Makefiles"
cp ./build-for-vscode/compile_commands.json .