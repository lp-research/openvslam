
#ifndef OPENVSLAM_EXPORT_H
#define OPENVSLAM_EXPORT_H

#ifdef OPENVSLAM_STATIC_DEFINE
#  define OPENVSLAM_EXPORT
#  define OPENVSLAM_NO_EXPORT
#else
#  ifndef OPENVSLAM_EXPORT
#    ifdef OPENVSLAM_EXPORTS
        /* We are building this library */
#      define OPENVSLAM_EXPORT __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define OPENVSLAM_EXPORT __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef OPENVSLAM_NO_EXPORT
#    define OPENVSLAM_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef OPENVSLAM_DEPRECATED
#  define OPENVSLAM_DEPRECATED __attribute__ ((__deprecated__))
#endif

#ifndef OPENVSLAM_DEPRECATED_EXPORT
#  define OPENVSLAM_DEPRECATED_EXPORT OPENVSLAM_EXPORT OPENVSLAM_DEPRECATED
#endif

#ifndef OPENVSLAM_DEPRECATED_NO_EXPORT
#  define OPENVSLAM_DEPRECATED_NO_EXPORT OPENVSLAM_NO_EXPORT OPENVSLAM_DEPRECATED
#endif

#if 0 /* DEFINE_NO_DEPRECATED */
#  ifndef OPENVSLAM_NO_DEPRECATED
#    define OPENVSLAM_NO_DEPRECATED
#  endif
#endif

#endif /* OPENVSLAM_EXPORT_H */
