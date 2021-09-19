set(COMPONENT_SRCS "main.c" "ui.c" "fft.c" "wifi.c" "ads1115.c" "dht.c")
set(COMPONENT_ADD_INCLUDEDIRS "." "./includes")

register_component()

target_add_binary_data(${COMPONENT_TARGET} "certs/aws-root-ca.pem" TEXT)