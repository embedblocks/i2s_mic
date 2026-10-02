# Unity tests for i2s_mic, packaged as a test component.
#
# Any ESP-IDF project that adds this directory to EXTRA_COMPONENT_DIRS
# (together with the i2s_mic component itself) gets these TEST_CASEs. The
# ready-made runner is ../test_app. WHOLE_ARCHIVE keeps the test objects in
# the link even though nothing calls them directly.
idf_component_register(SRC_DIRS "."
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES unity i2s_mic esp_timer freertos
                       WHOLE_ARCHIVE)
