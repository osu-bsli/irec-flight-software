target_sources(irec-flight-software PRIVATE
    SCS.cpp
    SCSCL.cpp
    SCSerial.cpp
    SMS_STS.cpp
)

target_include_directories(irec-flight-software PRIVATE
	.
)
