#include "gst_plugin.h"
#include "h264_encode.h"
#include "video_capture.h"

#include <gst/gst.h>
#include <gst/base/gstbasetransform.h>

#ifndef PACKAGE
#define PACKAGE "parrot_framework"
#endif

GST_DEBUG_CATEGORY_STATIC(gst_parrot_enc_debug);
#define GST_CAT_DEFAULT gst_parrot_enc_debug

/* Plugin specific caps */
#define PARROT_ENC_SRC_CAPS \
    "video/x-h264, " \
    "width=(int)[160,1280], " \
    "height=(int)[120,720], " \
    "framerate=(fraction)[1/1,30/1]"

#define PARROT_ENC_SINK_CAPS \
    "video/x-raw-yuv, " \
    "format=(fourcc)NV12, " \
    "width=(int)[160,1280], " \
    "height=(int)[120,720], " \
    "framerate=(fraction)[1/1,30/1]"

/* Element type */
typedef struct _GstParrotEnc {
    GstBaseTransform base;
    GstPad *sinkpad;
    GstPad *srcpad;

    h264_encoder_t encoder;
    int bitrate;
    int fps;
    int width;
    int height;
} GstParrotEnc;

typedef struct _GstParrotEncClass {
    GstBaseTransformClass parent_class;
} GstParrotEncClass;

enum {
    PROP_0,
    PROP_BITRATE,
    PROP_FPS,
};

/* Helper macros */
#define GST_TYPE_PARROT_ENC (gst_parrot_enc_get_type())
#define GST_PARROT_ENC(obj) (G_TYPE_CHECK_INSTANCE_CAST((obj), GST_TYPE_PARROT_ENC, GstParrotEnc))

GType gst_parrot_enc_get_type(void);

/* Forward declarations */
static void gst_parrot_enc_set_property(GObject *object, guint prop_id,
                                        const GValue *value, GParamSpec *pspec);
static void gst_parrot_enc_get_property(GObject *object, guint prop_id,
                                        GValue *value, GParamSpec *pspec);
static gboolean gst_parrot_enc_set_caps(GstBaseTransform *trans,
                                        GstCaps *incaps, GstCaps *outcaps);
static GstFlowReturn gst_parrot_enc_transform(GstBaseTransform *trans,
                                              GstBuffer *inbuf, GstBuffer *outbuf);

/* Boilerplate */
GST_BOILERPLATE(GstParrotEnc, gst_parrot_enc, GstBaseTransform,
                GST_TYPE_BASE_TRANSFORM);

static void gst_parrot_enc_base_init(gpointer g_class) {
    GstElementClass *element_class = GST_ELEMENT_CLASS(g_class);

    gst_element_class_set_details_simple(
        element_class,
        "Parrot H.264 Encoder",
        "Codec/Encoder/Video",
        "H.264 encoder using TI OMAP3530 DSP acceleration",
        "Parrot Framework Team");

    GstPadTemplate *sink_templ = gst_pad_template_new(
        "sink", GST_PAD_SINK, GST_PAD_ALWAYS,
        gst_caps_from_string(PARROT_ENC_SINK_CAPS));

    GstPadTemplate *src_templ = gst_pad_template_new(
        "src", GST_PAD_SRC, GST_PAD_ALWAYS,
        gst_caps_from_string(PARROT_ENC_SRC_CAPS));

    gst_element_class_add_pad_template(element_class, sink_templ);
    gst_element_class_add_pad_template(element_class, src_templ);
}

static void gst_parrot_enc_class_init(GstParrotEncClass *klass) {
    GObjectClass *gobject_class = G_OBJECT_CLASS(klass);
    GstBaseTransformClass *trans_class = GST_BASE_TRANSFORM_CLASS(klass);

    gobject_class->set_property = gst_parrot_enc_set_property;
    gobject_class->get_property = gst_parrot_enc_get_property;

    g_object_class_install_property(
        gobject_class, PROP_BITRATE,
        g_param_spec_int("bitrate", "Bitrate",
                         "Encoding bitrate in kbps",
                         100, 8000, 2000,
                         G_PARAM_READWRITE));

    g_object_class_install_property(
        gobject_class, PROP_FPS,
        g_param_spec_int("fps", "Frame rate",
                         "Encoding frame rate",
                         1, 30, 30,
                         G_PARAM_READWRITE));

    trans_class->set_caps = GST_DEBUG_FUNCPTR(gst_parrot_enc_set_caps);
    trans_class->transform = GST_DEBUG_FUNCPTR(gst_parrot_enc_transform);
}

static void gst_parrot_enc_init(GstParrotEnc *filter,
                                GstParrotEncClass *g_class) {
    (void)g_class;
    filter->bitrate = 2000;
    filter->fps = 30;
    filter->width = 0;
    filter->height = 0;
}

static void gst_parrot_enc_set_property(GObject *object, guint prop_id,
                                        const GValue *value, GParamSpec *pspec) {
    GstParrotEnc *filter = GST_PARROT_ENC(object);

    switch (prop_id) {
        case PROP_BITRATE:
            filter->bitrate = g_value_get_int(value);
            if (filter->encoder.initialized) {
                h264_encoder_set_bitrate(&filter->encoder, filter->bitrate);
            }
            break;
        case PROP_FPS:
            filter->fps = g_value_get_int(value);
            if (filter->encoder.initialized) {
                h264_encoder_set_framerate(&filter->encoder, filter->fps);
            }
            break;
        default:
            G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
            break;
    }
}

static void gst_parrot_enc_get_property(GObject *object, guint prop_id,
                                        GValue *value, GParamSpec *pspec) {
    GstParrotEnc *filter = GST_PARROT_ENC(object);

    switch (prop_id) {
        case PROP_BITRATE:
            g_value_set_int(value, filter->bitrate);
            break;
        case PROP_FPS:
            g_value_set_int(value, filter->fps);
            break;
        default:
            G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
            break;
    }
}

static gboolean gst_parrot_enc_set_caps(GstBaseTransform *trans,
                                        GstCaps *incaps, GstCaps *outcaps) {
    GstParrotEnc *filter = GST_PARROT_ENC(trans);
    (void)outcaps;

    // Parse input caps for resolution
    GstStructure *structure = gst_caps_get_structure(incaps, 0);
    gst_structure_get_int(structure, "width", &filter->width);
    gst_structure_get_int(structure, "height", &filter->height);

    // Open encoder with parsed parameters
    h264_encoder_open(&filter->encoder,
                      filter->width, filter->height,
                      filter->bitrate, filter->fps);

    return TRUE;
}

static GstFlowReturn gst_parrot_enc_transform(GstBaseTransform *trans,
                                              GstBuffer *inbuf, GstBuffer *outbuf) {
    GstParrotEnc *filter = GST_PARROT_ENC(trans);

    if (!filter->encoder.initialized) {
        GST_ERROR_OBJECT(filter, "Encoder not initialized");
        return GST_FLOW_NOT_NEGOTIATED;
    }

    // Get input data (NV12)
    guint8 *in_data = GST_BUFFER_DATA(inbuf);

    // Allocate output buffer
    guint out_size = filter->width * filter->height; // max
    guint8 *out_data = g_malloc(out_size);

    int h264_size = 0;
    int ret = h264_encoder_encode(&filter->encoder,
                                  in_data, out_data, &h264_size);

    if (ret < 0 || h264_size <= 0) {
        g_free(out_data);
        GST_ERROR_OBJECT(filter, "Encoding failed");
        return GST_FLOW_ERROR;
    }

    // Set output buffer
    GST_BUFFER_SIZE(outbuf) = h264_size;
    GST_BUFFER_DATA(outbuf) = out_data;

    return GST_FLOW_OK;
}

/* Plugin entry point */
static gboolean plugin_init(GstPlugin *plugin) {
    GST_DEBUG_CATEGORY_INIT(gst_parrot_enc_debug, "parrotenc", 0,
                            "Parrot AR.Drone H.264 Encoder");

    return gst_element_register(plugin, "parrotenc",
                                GST_RANK_PRIMARY,
                                GST_TYPE_PARROT_ENC);
}

GST_PLUGIN_DEFINE(
    GST_VERSION_MAJOR,
    GST_VERSION_MINOR,
    "parrot_enc",
    "H.264 encoder using OMAP3530 DSP acceleration",
    plugin_init,
    "1.0.0",
    "LGPL",
    "parrot_framework",
    "https://github.com/jsalinas/parrotFramework"
);
