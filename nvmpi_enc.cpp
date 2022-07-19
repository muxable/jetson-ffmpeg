#include "nvmpi.h"
#include "NvVideoEncoder.h"
#include "nvbuf_utils.h"
#include <vector>
#include <iostream>
#include <fcntl.h>
#include <thread>
#include <unistd.h>

#define CHUNK_SIZE 2*1024*1024
#define MAX_BUFFERS 32
#define TEST_ERROR(condition, message, errorCode)    \
	if (condition)                               \
{                                                    \
	std::cout<< message;                         \
}


using namespace std;

struct nvmpictx{
    NvVideoEncoder *enc;
    uint32_t encoder_pixfmt;
    uint32_t raw_pixfmt;

    uint32_t width;
    uint32_t height;

    uint32_t bitrate;
    uint32_t peak_bitrate;
    uint32_t profile;
    enum v4l2_mpeg_video_bitrate_mode ratecontrol;
    uint32_t iframe_interval;
    uint32_t idr_interval;
    uint32_t level;
    uint32_t fps_n;
    uint32_t fps_d;
    uint32_t gdr_start_frame_number; /* Frame number where GDR has to be started */
    uint32_t gdr_num_frames; /* Number of frames where GDR to be applied */
    uint32_t gdr_out_frame_number; /* Frames number from where encoded buffers are to be dumped */
    enum v4l2_enc_temporal_tradeoff_level_type temporal_tradeoff_level;
    enum v4l2_enc_hw_preset_type hw_preset_type;
    v4l2_enc_slice_length_type slice_length_type;
    uint32_t slice_length;
    uint32_t virtual_buffer_size;
    uint32_t num_reference_frames;
    uint32_t slice_intrarefresh_interval;
    uint32_t num_b_frames;
    uint32_t nMinQpI;              /* Minimum QP value to use for index frames */
    uint32_t nMaxQpI;              /* Maximum QP value to use for index frames */
    uint32_t nMinQpP;              /* Minimum QP value to use for P frames */
    uint32_t nMaxQpP;              /* Maximum QP value to use for P frames */
    uint32_t nMinQpB;              /* Minimum QP value to use for B frames */
    uint32_t nMaxQpB;              /* Maximum QP value to use for B frames */
    uint32_t sMaxQp;               /* Session Maximum QP value */
    uint32_t sar_width;
    uint32_t sar_height;
    uint8_t bit_depth;
    uint8_t chroma_format_idc;
    int output_plane_fd[32];
    bool insert_sps_pps_at_idr;
    bool enable_slice_level_encode;
    bool disable_cabac;
    bool insert_vui;
    bool enable_extended_colorformat;
    bool insert_aud;
    bool alliframes;
    bool is_semiplanar;
    enum v4l2_colorspace cs;

    bool input_metadata;
    uint32_t start_ts;
    bool enableGDR;
    bool bGapsInFrameNumAllowed;
    bool bnoIframe;
    uint32_t nH264FrameNumBits;
    uint32_t nH265PocLsbBits;
    bool externalRCHints;
    bool enableROI;
    bool b_use_enc_cmd;
    bool enableLossless;
    bool got_eos;

    bool externalRPS;
    bool RPS_threeLayerSvc;

    bool bReconCrc;
    uint32_t rl;                   /* Reconstructed surface Left cordinate */
    uint32_t rt;                   /* Reconstructed surface Top cordinate */
    uint32_t rw;                   /* Reconstructed surface width */
    uint32_t rh;                   /* Reconstructed surface height */

    uint32_t next_param_change_frame;
    int  stress_test;
    uint32_t endofstream_capture;
    uint32_t endofstream_output;

    uint32_t input_frames_queued_count;
    uint32_t num_output_buffers;
    int32_t num_frames_to_encode;
    uint32_t poc_type;

    int max_perf;


    int index;
    unsigned char *packet;
    uint32_t packet_size;
};

/**
  * Abort on error.
  *
  * @param ctx : Encoder context
  */
static void
abort(nvmpictx *ctx)
{
    ctx->enc->abort();
}

/**
  * Set encoder context defaults values.
  *
  * @param ctx : Encoder context
  */
static void
set_defaults(nvmpictx * ctx)
{
    memset(ctx, 0, sizeof(nvmpictx));

    ctx->raw_pixfmt = V4L2_PIX_FMT_YUV420M;
    ctx->bitrate = 4 * 1024 * 1024;
    ctx->peak_bitrate = 0;
    ctx->profile = V4L2_MPEG_VIDEO_H264_PROFILE_MAIN;
    ctx->ratecontrol = V4L2_MPEG_VIDEO_BITRATE_MODE_CBR;
    ctx->iframe_interval = 30;
    ctx->externalRPS = false;
    ctx->enableGDR = false;
    ctx->enableROI = false;
    ctx->bnoIframe = false;
    ctx->bGapsInFrameNumAllowed = false;
    ctx->bReconCrc = false;
    ctx->enableLossless = false;
    ctx->nH264FrameNumBits = 0;
    ctx->nH265PocLsbBits = 0;
    ctx->idr_interval = 256;
    ctx->level = -1;
    ctx->fps_n = 30;
    ctx->fps_d = 1;
    ctx->gdr_start_frame_number = 0xffffffff;
    ctx->gdr_num_frames = 0xffffffff;
    ctx->gdr_out_frame_number = 0xffffffff;
    ctx->num_b_frames = (uint32_t) -1;
    ctx->nMinQpI = (uint32_t)QP_RETAIN_VAL;
    ctx->nMaxQpI = (uint32_t)QP_RETAIN_VAL;
    ctx->nMinQpP = (uint32_t)QP_RETAIN_VAL;
    ctx->nMaxQpP = (uint32_t)QP_RETAIN_VAL;
    ctx->nMinQpB = (uint32_t)QP_RETAIN_VAL;
    ctx->nMaxQpB = (uint32_t)QP_RETAIN_VAL;
    ctx->externalRCHints = false;
    ctx->input_metadata = false;
    ctx->sMaxQp = 51;
    ctx->stress_test = 1;
    ctx->cs = V4L2_COLORSPACE_SMPTE170M;
    ctx->sar_width = 0;
    ctx->sar_height = 0;
    ctx->start_ts = 0;
    ctx->max_perf = 0;
    ctx->num_output_buffers = 6;
    ctx->num_frames_to_encode = -1;
    ctx->poc_type = 0;
    ctx->chroma_format_idc = -1;
    ctx->bit_depth = 8;
    ctx->is_semiplanar = false;
}

nvmpictx* nvmpi_create_encoder(nvCodingType codingType,nvEncParam * param){

    int ret = 0;
	log_level = LOG_LEVEL_INFO;
	nvmpictx* ctx = new nvmpictx;
    /* Set default values for encoder context members. */
    set_defaults(ctx);

    // configuration overrides.
	ctx->width = param->width;
	ctx->height = param->height;
	if(codingType==NV_VIDEO_CodingH264){
		ctx->encoder_pixfmt=V4L2_PIX_FMT_H264;
	}else if(codingType==NV_VIDEO_CodingHEVC){
		ctx->encoder_pixfmt=V4L2_PIX_FMT_H265;
	}

    if (ctx->encoder_pixfmt == V4L2_PIX_FMT_H265)
    {
        TEST_ERROR(ctx->width < 144 || ctx->height < 144, "Height/Width should be"
            " > 144 for H.265", ret);
    }

    ctx->max_perf = 1;
	ctx->fps_n = param->fps_n;
	ctx->fps_d = param->fps_d;
	ctx->num_b_frames=param->max_b_frames;
	ctx->idr_interval = param->idr_interval;
	ctx->iframe_interval = param->iframe_interval;

    /* Create NvVideoEncoder object for blocking I/O mode. */
	ctx->enc = NvVideoEncoder::createVideoEncoder("enc0", O_NONBLOCK);
    TEST_ERROR(!ctx->enc, "Could not create encoder", ret);

    /* Set encoder capture plane format.
       NOTE: It is necessary that Capture Plane format be set before Output Plane
       format. It is necessary to set width and height on the capture plane as well */
    ret =
        ctx->enc->setCapturePlaneFormat(ctx->encoder_pixfmt, ctx->width,
                                      ctx->height, 2 * 1024 * 1024);
    TEST_ERROR(ret < 0, "Could not set capture plane format", ret);

    if (ctx->encoder_pixfmt == V4L2_PIX_FMT_H265)
    {
        switch (ctx->profile)
        {
            case V4L2_MPEG_VIDEO_H265_PROFILE_MAIN10:
            {
                ctx->raw_pixfmt = V4L2_PIX_FMT_P010M;
                ctx->is_semiplanar = true; /* To keep previous execution commands working */
                ctx->bit_depth = 10;
                break;
            }
            case V4L2_MPEG_VIDEO_H265_PROFILE_MAIN:
            {
                if (ctx->is_semiplanar)
                    ctx->raw_pixfmt = V4L2_PIX_FMT_NV12M;
                else
                    ctx->raw_pixfmt = V4L2_PIX_FMT_YUV420M;
                if (ctx->chroma_format_idc == 3)
                {
                    if (ctx->bit_depth == 10 && ctx->is_semiplanar)
                        ctx->raw_pixfmt = V4L2_PIX_FMT_NV24_10LE;
                    if (ctx->bit_depth == 8)
                    {
                        if (ctx->is_semiplanar)
                            ctx->raw_pixfmt = V4L2_PIX_FMT_NV24M;
                        else
                            ctx->raw_pixfmt = V4L2_PIX_FMT_YUV444M;
                    }
                }
            }
                break;
            default:
                ctx->raw_pixfmt = V4L2_PIX_FMT_YUV420M;
        }
    }
    if (ctx->encoder_pixfmt == V4L2_PIX_FMT_H264)
    {
        if (ctx->enableLossless &&
            ctx->profile == V4L2_MPEG_VIDEO_H264_PROFILE_HIGH_444_PREDICTIVE)
        {
            if (ctx->is_semiplanar)
                ctx->raw_pixfmt = V4L2_PIX_FMT_NV24M;
            else
                ctx->raw_pixfmt = V4L2_PIX_FMT_YUV444M;
        }
        else if ((ctx->enableLossless &&
            ctx->profile != V4L2_MPEG_VIDEO_H264_PROFILE_HIGH_444_PREDICTIVE) ||
            (!ctx->enableLossless && ctx->profile == V4L2_MPEG_VIDEO_H264_PROFILE_HIGH_444_PREDICTIVE))
        {
			TEST_ERROR(true, "Lossless encoding is supported only for high444 profile", -1);
        }
        else
        {
            if (ctx->is_semiplanar)
                ctx->raw_pixfmt = V4L2_PIX_FMT_NV12M;
            else
                ctx->raw_pixfmt = V4L2_PIX_FMT_YUV420M;
        }
    }

    /* Set encoder output plane format */
    ret =
        ctx->enc->setOutputPlaneFormat(ctx->raw_pixfmt, ctx->width,
                                      ctx->height);
    TEST_ERROR(ret < 0, "Could not set output plane format", ret);

    ret = ctx->enc->setBitrate(ctx->bitrate);
    TEST_ERROR(ret < 0, "Could not set encoder bitrate", ret);

    if (ctx->encoder_pixfmt == V4L2_PIX_FMT_H264)
    {
        /* Set encoder profile for H264 format */
        ret = ctx->enc->setProfile(ctx->profile);
        TEST_ERROR(ret < 0, "Could not set encoder profile", ret);

        if (ctx->level == (uint32_t)-1)
        {
            ctx->level = (uint32_t)V4L2_MPEG_VIDEO_H264_LEVEL_5_1;
        }

        /* Set encoder level for H264 format */
        ret = ctx->enc->setLevel(ctx->level);
        TEST_ERROR(ret < 0, "Could not set encoder level", ret);
    }
    else if (ctx->encoder_pixfmt == V4L2_PIX_FMT_H265)
    {
        /* Set encoder profile for HEVC format */
        ret = ctx->enc->setProfile(ctx->profile);
        TEST_ERROR(ret < 0, "Could not set encoder profile", ret);

        if (ctx->level != (uint32_t)-1)
        {
            /* Set encoder level for HEVC format */
            ret = ctx->enc->setLevel(ctx->level);
            TEST_ERROR(ret < 0, "Could not set encoder level", ret);
        }

        if (ctx->chroma_format_idc != (uint8_t)-1)
        {
            ret = ctx->enc->setChromaFactorIDC(ctx->chroma_format_idc);
            TEST_ERROR(ret < 0, "Could not set chroma_format_idc", ret);
        }
    }

    if (ctx->enableLossless)
    {
        ret = ctx->enc->setLossless(ctx->enableLossless);
        TEST_ERROR(ret < 0, "Could not set lossless encoding", ret);
    }
    else
    {
        /* Set rate control mode for encoder */
        ret = ctx->enc->setRateControlMode(ctx->ratecontrol);
        TEST_ERROR(ret < 0, "Could not set encoder rate control mode", ret);
        if (ctx->ratecontrol == V4L2_MPEG_VIDEO_BITRATE_MODE_VBR) {
            uint32_t peak_bitrate;
            if (ctx->peak_bitrate < ctx->bitrate)
                peak_bitrate = 1.2f * ctx->bitrate;
            else
                peak_bitrate = ctx->peak_bitrate;
            /* Set peak bitrate value for variable bitrate mode for encoder */
            ret = ctx->enc->setPeakBitrate(peak_bitrate);
            TEST_ERROR(ret < 0, "Could not set encoder peak bitrate", ret);
        }
    }

    if (ctx->poc_type)
    {
        ret = ctx->enc->setPocType(ctx->poc_type);
        TEST_ERROR(ret < 0, "Could not set Picture Order Count value", ret);
    }

    /* Set IDR frame interval for encoder */
    ret = ctx->enc->setIDRInterval(ctx->idr_interval);
    TEST_ERROR(ret < 0, "Could not set encoder IDR interval", ret);

    /* Set I frame interval for encoder */
    ret = ctx->enc->setIFrameInterval(ctx->iframe_interval);
    TEST_ERROR(ret < 0, "Could not set encoder I-Frame interval", ret);

    /* Set framerate for encoder */
    ret = ctx->enc->setFrameRate(ctx->fps_n, ctx->fps_d);
    TEST_ERROR(ret < 0, "Could not set framerate", ret);

    if (ctx->temporal_tradeoff_level)
    {
        /* Set temporal tradeoff level value for encoder */
        ret = ctx->enc->setTemporalTradeoff(ctx->temporal_tradeoff_level);
        TEST_ERROR(ret < 0, "Could not set temporal tradeoff level", ret);
    }

    if (ctx->slice_length)
    {
        /* Set slice length value for encoder */
        ret = ctx->enc->setSliceLength(ctx->slice_length_type,
                ctx->slice_length);
        TEST_ERROR(ret < 0, "Could not set slice length params", ret);
    }

    if (ctx->enable_slice_level_encode)
    {
        /* Enable slice level encode for encoder */
        ret = ctx->enc->setSliceLevelEncode(true);
        TEST_ERROR(ret < 0, "Could not set slice level encode", ret);
    }

    if (ctx->hw_preset_type)
    {
        /* Set hardware preset value for encoder */
        ret = ctx->enc->setHWPresetType(ctx->hw_preset_type);
        TEST_ERROR(ret < 0, "Could not set encoder HW Preset Type", ret);
    }

    if (ctx->virtual_buffer_size)
    {
        /* Set virtual buffer size value for encoder */
        ret = ctx->enc->setVirtualBufferSize(ctx->virtual_buffer_size);
        TEST_ERROR(ret < 0, "Could not set virtual buffer size", ret);
    }

    if (ctx->slice_intrarefresh_interval)
    {
        /* Set slice intra refresh interval value for encoder */
        ret = ctx->enc->setSliceIntrarefresh(ctx->slice_intrarefresh_interval);
        TEST_ERROR(ret < 0, "Could not set slice intrarefresh interval", ret);
    }

    if (ctx->insert_sps_pps_at_idr)
    {
        /* Enable insert of SPSPPS at IDR frames */
        ret = ctx->enc->setInsertSpsPpsAtIdrEnabled(true);
        TEST_ERROR(ret < 0, "Could not set insertSPSPPSAtIDR", ret);
    }

    if (ctx->disable_cabac)
    {
        /* Disable CABAC entropy encoding */
        ret = ctx->enc->setCABAC(false);
        TEST_ERROR(ret < 0, "Could not set disable CABAC", ret);
    }

    if (ctx->sar_width)
    {
        /* Set SAR width */
        ret = ctx->enc->setSampleAspectRatioWidth(ctx->sar_width);
        TEST_ERROR(ret < 0, "Could not set Sample Aspect Ratio width", ret);
    }

    if (ctx->sar_height)
    {
        /* Set SAR width */
        ret = ctx->enc->setSampleAspectRatioHeight(ctx->sar_height);
        TEST_ERROR(ret < 0, "Could not set Sample Aspect Ratio height", ret);
    }

    if (ctx->insert_vui)
    {
        /* Enable insert of VUI parameters */
        ret = ctx->enc->setInsertVuiEnabled(true);
        TEST_ERROR(ret < 0, "Could not set insertVUI", ret);
    }

    if (ctx->enable_extended_colorformat)
    {
        /* Enable extnded colorformat for encoder */
        ret = ctx->enc->setExtendedColorFormat(true);
        TEST_ERROR(ret < 0, "Could not set extended color format", ret);
    }

    if (ctx->insert_aud)
    {
        /* Enable insert of AUD parameters */
        ret = ctx->enc->setInsertAudEnabled(true);
        TEST_ERROR(ret < 0, "Could not set insertAUD", ret);
    }

    if (ctx->alliframes)
    {
        /* Enable all I-frame encode */
        ret = ctx->enc->setAlliFramesEncode(true);
        TEST_ERROR(ret < 0, "Could not set Alliframes encoding", ret);
    }

    if (ctx->num_b_frames != (uint32_t) -1)
    {
        /* Set number of B-frames to to be used by encoder */
        ret = ctx->enc->setNumBFrames(ctx->num_b_frames);
        TEST_ERROR(ret < 0, "Could not set number of B Frames", ret);
    }

    if ((ctx->nMinQpI != (uint32_t)QP_RETAIN_VAL) ||
        (ctx->nMaxQpI != (uint32_t)QP_RETAIN_VAL) ||
        (ctx->nMinQpP != (uint32_t)QP_RETAIN_VAL) ||
        (ctx->nMaxQpP != (uint32_t)QP_RETAIN_VAL) ||
        (ctx->nMinQpB != (uint32_t)QP_RETAIN_VAL) ||
        (ctx->nMaxQpB != (uint32_t)QP_RETAIN_VAL))
    {
        /* Set Min & Max qp range values for I/P/B-frames to be used by encoder */
        ret = ctx->enc->setQpRange(ctx->nMinQpI, ctx->nMaxQpI, ctx->nMinQpP,
                ctx->nMaxQpP, ctx->nMinQpB, ctx->nMaxQpB);
        TEST_ERROR(ret < 0, "Could not set quantization parameters", ret);
    }

    if (ctx->max_perf)
    {
        /* Enable maximum performance mode by disabling internal DFS logic.
           NOTE: This enables encoder to run at max clocks */
        ret = ctx->enc->setMaxPerfMode(ctx->max_perf);
        TEST_ERROR(ret < 0, "Error while setting encoder to max perf", ret);
    }

    if (ctx->bnoIframe) {
        ctx->iframe_interval = ((1<<31) + 1); /* TODO: how can we do this properly */
        ret = ctx->enc->setIFrameInterval(ctx->iframe_interval);
        TEST_ERROR(ret < 0, "Could not set encoder I-Frame interval", ret);
    }

    if (ctx->enableROI) {
        v4l2_enc_enable_roi_param VEnc_enable_ext_roi_ctrl;

        VEnc_enable_ext_roi_ctrl.bEnableROI = ctx->enableROI;
        /* Enable region of intrest configuration for encoder */
        ret = ctx->enc->enableROI(VEnc_enable_ext_roi_ctrl);
        TEST_ERROR(ret < 0, "Could not enable ROI", ret);
    }

    if (ctx->bReconCrc) {
        v4l2_enc_enable_reconcrc_param VEnc_enable_recon_crc_ctrl;

        VEnc_enable_recon_crc_ctrl.bEnableReconCRC = ctx->bReconCrc;
        /* Enable reconstructed CRC configuration for encoder */
        ret = ctx->enc->enableReconCRC(VEnc_enable_recon_crc_ctrl);
        TEST_ERROR(ret < 0, "Could not enable Recon CRC", ret);
    }

    if (ctx->externalRPS) {
        v4l2_enc_enable_ext_rps_ctr VEnc_enable_ext_rps_ctrl;

        VEnc_enable_ext_rps_ctrl.bEnableExternalRPS = ctx->externalRPS;
        if (ctx->encoder_pixfmt == V4L2_PIX_FMT_H264) {
            VEnc_enable_ext_rps_ctrl.bGapsInFrameNumAllowed = ctx->bGapsInFrameNumAllowed;
            VEnc_enable_ext_rps_ctrl.nH264FrameNumBits = ctx->nH264FrameNumBits;
        }
        if (ctx->encoder_pixfmt == V4L2_PIX_FMT_H265) {
            VEnc_enable_ext_rps_ctrl.nH265PocLsbBits = ctx->nH265PocLsbBits;
        }
        /* Enable external reference picture set configuration for encoder */
        ret = ctx->enc->enableExternalRPS(VEnc_enable_ext_rps_ctrl);
        TEST_ERROR(ret < 0, "Could not enable external RPS", ret);
    }

    if (ctx->num_reference_frames)
    {
        /* Set number of reference frame configuration value for encoder */
        ret = ctx->enc->setNumReferenceFrames(ctx->num_reference_frames);
        TEST_ERROR(ret < 0, "Could not set num reference frames", ret);
    }

    if (ctx->externalRCHints) {
        v4l2_enc_enable_ext_rate_ctr VEnc_enable_ext_rate_ctrl;

        VEnc_enable_ext_rate_ctrl.bEnableExternalPictureRC = ctx->externalRCHints;
        VEnc_enable_ext_rate_ctrl.nsessionMaxQP = ctx->sMaxQp;

        /* Enable external rate control configuration for encoder */
        ret = ctx->enc->enableExternalRC(VEnc_enable_ext_rate_ctrl);
        TEST_ERROR(ret < 0, "Could not enable external RC", ret);
    }

    /* Query, Export and Map the output plane buffers so that we can read
       raw data into the buffers */
    ret = ctx->enc->output_plane.setupPlane(V4L2_MEMORY_USERPTR, 10, false, true);
    TEST_ERROR(ret < 0, "Could not setup output plane", ret);

    /* Query, Export and Map the capture plane buffers so that we can write
       encoded bitstream data into the buffers */
    ret = ctx->enc->capture_plane.setupPlane(V4L2_MEMORY_MMAP, ctx->num_output_buffers,
        true, false);
    TEST_ERROR(ret < 0, "Could not setup capture plane", ret);

    /* Subscibe for End Of Stream event */
    ret = ctx->enc->subscribeEvent(V4L2_EVENT_EOS,0,0);
    TEST_ERROR(ret < 0, "Could not subscribe EOS event", ret);

	/* set encoder output plane STREAMON */
	ret = ctx->enc->output_plane.setStreamStatus(true);
	TEST_ERROR(ret < 0, "Error in output plane streamon", ret);

	/* set encoder capture plane STREAMON */
	ret = ctx->enc->capture_plane.setStreamStatus(true);
	TEST_ERROR(ret < 0, "Error in capture plane streamon", ret);

    /* Enqueue all the empty capture plane buffers. */
    for (uint32_t i = 0; i < ctx->enc->capture_plane.getNumBuffers(); i++)
    {
        struct v4l2_buffer v4l2_buf;
        struct v4l2_plane planes[MAX_PLANES];

        memset(&v4l2_buf, 0, sizeof(v4l2_buf));
        memset(planes, 0, MAX_PLANES * sizeof(struct v4l2_plane));

        v4l2_buf.index = i;
        v4l2_buf.m.planes = planes;

        ret = ctx->enc->capture_plane.qBuffer(v4l2_buf, NULL);
		TEST_ERROR(ret < 0, "Error while queueing buffer at capture plane", ret);
    }

	return ctx;
}


int nvmpi_encoder_put_frame(nvmpictx* ctx,nvFrame* frame){
	struct v4l2_buffer v4l2_output_buf;
	struct v4l2_plane output_planes[MAX_PLANES];
	NvBuffer *outplane_buffer = NULL;
	int ret;

	memset(&v4l2_output_buf, 0, sizeof(v4l2_output_buf));
	memset(output_planes, 0, sizeof(output_planes));
	v4l2_output_buf.m.planes = output_planes;

	if(ctx->enc->isInError()) {
		return -1;
    }

	if(ctx->index < ctx->enc->output_plane.getNumBuffers()){
		outplane_buffer=ctx->enc->output_plane.getNthBuffer(ctx->index);
		v4l2_output_buf.index = ctx->index ;
		ctx->index++;

	}else{
		ret = ctx->enc->output_plane.dqBuffer(v4l2_output_buf, &outplane_buffer, NULL, -1);
		if (ret < 0) {
			cout << "Error DQing buffer at output plane" << std::endl;
			return false;
		}
	}

	memcpy(outplane_buffer->planes[0].data,frame->payload[0],frame->payload_size[0]);
	memcpy(outplane_buffer->planes[1].data,frame->payload[1],frame->payload_size[1]);
	memcpy(outplane_buffer->planes[2].data,frame->payload[2],frame->payload_size[2]);
	outplane_buffer->planes[0].bytesused=frame->payload_size[0];
	outplane_buffer->planes[1].bytesused=frame->payload_size[1];
	outplane_buffer->planes[2].bytesused=frame->payload_size[2];

	v4l2_output_buf.flags |= V4L2_BUF_FLAG_TIMESTAMP_COPY;
	v4l2_output_buf.timestamp.tv_usec = frame->timestamp % 1000000;
	v4l2_output_buf.timestamp.tv_sec = frame->timestamp / 1000000;

	/* encoder qbuffer for output plane */
	ret = ctx->enc->output_plane.qBuffer(v4l2_output_buf, NULL);
	if (ret < 0)
	{
		cerr << "Error while queueing buffer at output plane" << endl;
		ctx->enc->abort();
		return -1;
	}

	return 0;
}

int nvmpi_encoder_get_packet(nvmpictx* ctx,nvPacket* packet){
	struct v4l2_buffer v4l2_capture_buf;
	struct v4l2_plane capture_planes[MAX_PLANES];
	NvBuffer *capplane_buffer = NULL;
	bool capture_dq_continue = true;
    NvVideoEncoder *enc = ctx->enc;
    uint32_t frame_num = ctx->enc->capture_plane.getTotalDequeuedBuffers() - 1;
    uint32_t ReconRef_Y_CRC = 0;
    uint32_t ReconRef_U_CRC = 0;
    uint32_t ReconRef_V_CRC = 0;
    static uint32_t num_encoded_frames = 1;
    struct v4l2_event ev;
    int ret = 0;

	memset(&v4l2_capture_buf, 0, sizeof(v4l2_capture_buf));
	memset(capture_planes, 0, sizeof(capture_planes));
	v4l2_capture_buf.m.planes = capture_planes;
	v4l2_capture_buf.length = 1;

	/* Dequeue from output plane, fill the frame and enqueue it back again.
		NOTE: This could be moved out to a different thread as an optimization. */
	ret = ctx->enc->capture_plane.dqBuffer(v4l2_capture_buf, &capplane_buffer, NULL, 10);
	if (ret < 0)
	{
		if (errno == EAGAIN) {
			return -1;
		}
		cerr << "ERROR while DQing buffer at capture plane" << endl;
		ctx->enc->abort();
		return -1;
	}

    /* Received EOS from encoder. Stop dqthread. */
    if (capplane_buffer->planes[0].bytesused == 0)
    {
        cout << "Got 0 size buffer in capture \n";
        return -1;
    }
	auto size = capplane_buffer->planes[0].bytesused;

	if(ctx->packet_size < size){

		ctx->packet_size=size;

        delete ctx->packet;
        ctx->packet=new unsigned char[ctx->packet_size];	
	}

	auto ts = (v4l2_capture_buf.timestamp.tv_usec % 1000000) + (v4l2_capture_buf.timestamp.tv_sec * 1000000UL);

	if((ts > 0) && (size == 0)) // Old packet, but 0-0 skip!
	{
		return -1;
	}
    
	memcpy(ctx->packet,capplane_buffer->planes[0].data,size);
    packet->payload = ctx->packet;
    packet->payload_size = size;

	packet->pts = ts;
	v4l2_ctrl_videoenc_outputbuf_metadata enc_metadata;
	ctx->enc->getMetadata(v4l2_capture_buf.index, enc_metadata);
	if(enc_metadata.KeyFrame){
		packet->flags|= 0x0001;//AV_PKT_FLAG_KEY 0x0001
	}

    /* encoder qbuffer for capture plane */
    if (enc->capture_plane.qBuffer(v4l2_capture_buf, NULL) < 0)
    {
        cerr << "Error while Qing buffer at capture plane" << endl;
        return false;
    }

	return 0;
}

int nvmpi_encoder_set_bitrate(nvmpictx* ctx, int intval) {
	int ret;

    if (ctx->ratecontrol == V4L2_MPEG_VIDEO_BITRATE_MODE_VBR &&
        ctx->peak_bitrate < intval) {
        uint32_t peak_bitrate = 1.2f * intval;
        cout << "Peak bitrate = " << peak_bitrate << endl;
        ret = ctx->enc->setPeakBitrate(peak_bitrate);
        if (ret < 0)
        {
            cerr << "Could not set encoder peakbitrate" << endl;
            return -1;
        }
    }
    cout << "Bitrate = " << intval << endl;
    ret = ctx->enc->setBitrate(intval);
    if (ret < 0)
    {
        cerr << "Could not set encoder bitrate" << endl;
        return -1;
    }

	return 0;
}

int nvmpi_encoder_force_idr(nvmpictx* ctx) {
	return ctx->enc->forceIDR();
}

int nvmpi_encoder_close(nvmpictx* ctx){
	int ret;

    /* Release encoder configuration specific resources. */
    delete ctx->enc;
    delete ctx->packet;

	delete ctx;

	return ret;
}

