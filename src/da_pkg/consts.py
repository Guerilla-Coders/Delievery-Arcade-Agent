class Limits:
    WAFFLE_MAX_LIN_VEL = 0.26
    WAFFLE_MAX_ANG_VEL = 1.82

    LIN_VEL_STEP_SIZE = 0.01
    ANG_VEL_STEP_SIZE = 0.1

    status = 0
    target_linear_vel = 0.0
    target_angular_vel = 0.0

    control_linear_vel = 0.0
    control_angular_vel = 0.0


class SoundEffectConstants:
    silence = "SILENCE"
    greeting = "GREETING"
    apology = "APOLOGY"
    appreciation = "APPRECIATION"
    yield_request = "YIELD"
    alert = "ALERT"
    pigeon = "PIGEON"

    english = "EN"
    korean = "KR"
    spanish = "ES"

    code = ["GREETING", "APOLOGY", "APPRECIATION", "YIELD", "ALERT", "PIGEON"]
    speech_needed_code = ["GREETING", "APOLOGY", "APPRECIATION", "YIELD"]
    audio_needed_code = ["ALERT", "PIGEON"]
    language = ["EN", "KR", "ES"]


class SpeechData:
    lines = {
        SoundEffectConstants.english:
            {
                SoundEffectConstants.greeting: ["Nice to meet you, I am here with your foods",
                                                "Finally! A worthy opponent! Our battle will be legendery!"],
                SoundEffectConstants.apology: [
                    "I am so sorry for my mistakes! It won't happen again.",
                    "My apologies. Could you please forgive me for that?"
                ],
                SoundEffectConstants.appreciation: [
                    "Thank you so much.",
                    "Thanks. My owner also wants me to send his best appreciation to you."
                ],
                SoundEffectConstants.yield_request: [
                    "Sir, I am sorry to bother you, but could you step aside please?",
                    "Sir, could you forgive me for asking you to step aside please? I am in a hurry."
                ]
            }
    }


class ActionConstants:
    lid_open = "OPEN"
    lid_close = "CLOSE"
