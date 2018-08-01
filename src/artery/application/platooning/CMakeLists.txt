#######################################
# Now the actual application is built #
#######################################

add_artery_feature(platooningservice)

# Now that all sources are ready, we can add them to the target
target_sources(platooningservice PRIVATE
	apps/BaseApp.cc
	apps/SimplePlatooningApp.cc
    protocols/BaseProtocol.cc
    protocols/SimplePlatooningBeaconing.cc
    protocols/SlottedBeaconing.cc
    scenarios/AccelerateAndBrakeScenario.cc
    scenarios/BaseScenario.cc
    scenarios/BrakingScenario.cc
    scenarios/SimpleScenario.cc
    scenarios/SinusoidalScenario.cc
    utilities/PositionHelper.cc
    utilities/BasePositionHelper.cc
    utilities/PlatooningBaseModule.cc
    PlatooningService.cc
    UnicastProtocol.cc
)

generate_opp_message(messages/InterferingBeacon.msg TARGET platooningservice)
generate_opp_message(messages/PlatooningBeacon.msg TARGET platooningservice)
generate_opp_message(messages/UnicastMessage.msg TARGET platooningservice)
generate_opp_message(messages/UnicastProtocolControlMessage.msg TARGET platooningservice)