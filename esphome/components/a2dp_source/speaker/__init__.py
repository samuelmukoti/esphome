import esphome.codegen as cg
from esphome.components import audio, speaker
import esphome.config_validation as cv
from esphome.const import (
    CONF_BITS_PER_SAMPLE,
    CONF_BUFFER_DURATION,
    CONF_ID,
    CONF_NUM_CHANNELS,
    CONF_SAMPLE_RATE,
)
import esphome.final_validate as fv
from esphome.types import ConfigType

from .. import CONF_A2DP_SOURCE_ID, DOMAIN, A2DPSource, a2dp_source_ns

CODEOWNERS = ["@samuelmukoti"]
DEPENDENCIES = [DOMAIN]

A2DPSourceSpeaker = a2dp_source_ns.class_(
    "A2DPSourceSpeaker", cg.Component, speaker.Speaker
)

# The Bluetooth stack's SBC encoder only takes 16-bit PCM at 44.1 kHz
A2DP_SAMPLE_RATE = 44100
A2DP_BITS_PER_SAMPLE = 16


def _set_stream_limits(config: ConfigType) -> ConfigType:
    audio.set_stream_limits(
        min_bits_per_sample=A2DP_BITS_PER_SAMPLE,
        max_bits_per_sample=A2DP_BITS_PER_SAMPLE,
        min_channels=1,
        max_channels=2,
        min_sample_rate=A2DP_SAMPLE_RATE,
        max_sample_rate=A2DP_SAMPLE_RATE,
    )(config)
    return config


CONFIG_SCHEMA = cv.All(
    speaker.SPEAKER_SCHEMA.extend(
        {
            cv.GenerateID(): cv.declare_id(A2DPSourceSpeaker),
            cv.GenerateID(CONF_A2DP_SOURCE_ID): cv.use_id(A2DPSource),
            cv.Optional(CONF_BUFFER_DURATION, default="100ms"): cv.All(
                cv.positive_time_period_milliseconds,
                cv.Range(min=cv.TimePeriod(milliseconds=20)),
            ),
            # Set so a media player inherits the format the stream takes
            cv.Optional(CONF_SAMPLE_RATE, default=A2DP_SAMPLE_RATE): cv.one_of(
                A2DP_SAMPLE_RATE, int=True
            ),
            cv.Optional(CONF_BITS_PER_SAMPLE, default=A2DP_BITS_PER_SAMPLE): cv.one_of(
                A2DP_BITS_PER_SAMPLE, int=True
            ),
            cv.Optional(CONF_NUM_CHANNELS, default=2): cv.int_range(min=1, max=2),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    _set_stream_limits,
)

FINAL_VALIDATE_SCHEMA = fv.unique_platform_key(
    domain="speaker",
    platform=DOMAIN,
    message="Only one a2dp_source speaker is supported",
)


async def to_code(config: ConfigType) -> None:
    var = cg.new_Pvariable(
        config[CONF_ID],
        config[CONF_BUFFER_DURATION].total_milliseconds,
        config[CONF_NUM_CHANNELS],
    )
    await cg.register_component(var, config)
    await speaker.register_speaker(var, config)
    hub = await cg.get_variable(config[CONF_A2DP_SOURCE_ID])
    await cg.register_parented(var, hub)
    cg.add(hub.set_stream(var))
