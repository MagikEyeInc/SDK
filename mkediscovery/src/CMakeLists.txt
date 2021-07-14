set(SRC_FILES
  main.cpp
  mainwindow.cpp
  aboutdialog.cpp
  devicelist.cpp
  ../resources/resources.qrc
  ../resources/resources.rc
)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  set(APP_NAME "MkEDiscovery")
  ADD_EXECUTABLE(${PROJECT_NAME} MACOSX_BUNDLE ${SRC_FILES})
  set(ICON_NAME "mkelogo.icns")
  set(ICON_PATH "${CMAKE_SOURCE_DIR}/resources/${ICON_NAME}")
  set_target_properties(${PROJECT_NAME} PROPERTIES 
    MACOSX_BUNDLE_BUNDLE_NAME ${APP_NAME}
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION}
    MACOSX_BUNDLE_ICONFILE ${ICON_NAME}
    MACOSX_BUNDLE_INFO_PLIST ${CMAKE_SOURCE_DIR}/cmake/MacOSXBundleInfo.plist.in
  )

else()
  set(APP_NAME "${PROJECT_NAME}")  
  set(APP_ICON_RESOURCE_WINDOWS "${CMAKE_SOURCE_DIR}/resources/resources.rc")
  ADD_EXECUTABLE(${APP_NAME} ${SRC_FILES})
endif()

# Do we need to compile libmkeclient?
if(MKECLI_TARGET)
  add_dependencies(${PROJECT_NAME} ${MKECLI_TARGET})
endif()

# Make 'src' the include root
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} PRIVATE ".")

# libMkE
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} PUBLIC ${MKECLI_INCLUDE_DIRS})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} PRIVATE ${MKECLI_LIBRARIES})

# Qt libraries
TARGET_LINK_LIBRARIES(${PROJECT_NAME} PRIVATE
  Qt5::Widgets
)

# Pthreads
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE Threads::Threads)

# Windows specific
IF(WIN32)
  SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-subsystem,windows")

  TARGET_LINK_LIBRARIES(${PROJECT_NAME} PRIVATE
    wsock32 
    ws2_32
    iphlpapi
  )
ENDIF() 

# MacOS specific
if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  target_link_libraries(${PROJECT_NAME} PRIVATE "-framework SystemConfiguration -framework Carbon")
endif()


###############################################################################
# Packages

IF(CMAKE_SYSTEM_NAME MATCHES "Linux")
  add_custom_target(dist_package
    DEPENDS ${PROJECT_NAME}
    COMMAND cd "${CMAKE_BINARY_DIR}" && linuxdeployqt ${CMAKE_BINARY_DIR}/src/${APP_NAME} -appimage
    COMMAND mv "${CMAKE_BINARY_DIR}/Application-x86_64.AppImage" "${CMAKE_BINARY_DIR}/${APP_NAME}-${PROJECT_VERSION}-x86_64.AppImage"
  )
ENDIF()

####

IF(WIN32)
  set(EXE_NAME "${APP_NAME}.exe")
  set(INSTALL_DIR ${CMAKE_BINARY_DIR}/${APP_NAME})
  
  add_custom_target(dist_package
    DEPENDS ${PROJECT_NAME}
	  COMMAND ${CMAKE_COMMAND} -E make_directory "${INSTALL_DIR}"
	  COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_BINARY_DIR}/src/${EXE_NAME}" "${INSTALL_DIR}"
	  COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_SOURCE_DIR}/LICENSE.txt" "${INSTALL_DIR}"
      COMMAND windeployqt --release --no-opengl-sw --no-system-d3d-compiler --no-webkit2 --no-angle --no-translations  "${INSTALL_DIR}/${EXE_NAME}"
	  COMMAND 7z a -tzip "${CMAKE_BINARY_DIR}/${APP_NAME}-${PROJECT_VERSION}.zip" -r ${INSTALL_DIR}
	  COMMAND "C://Program Files (x86)/Inno Setup 6/iscc.exe" /DMyAppInstallerName="${APP_NAME}setup-${PROJECT_VERSION}" /DMyAppVersion="${PROJECT_VERSION}" /DMyAppName="${APP_NAME}" /DMyAppExeName="${EXE_NAME}" /DMyAppBuildDir="${INSTALL_DIR}/" "${CMAKE_SOURCE_DIR}/installer_win/installer_script.iss" 
  )
ENDIF()

####

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  SET(APP_PATH "${CMAKE_BINARY_DIR}/src/${APP_NAME}.app/")
  set(RESOURCES_DIR "${APP_PATH}/Contents/Resources/")

  add_custom_target(dist_package
    DEPENDS ${PROJECT_NAME}
    COMMAND ${CMAKE_COMMAND} -E make_directory ${RESOURCES_DIR}
    COMMAND ${CMAKE_COMMAND} -E copy ${ICON_PATH} ${RESOURCES_DIR}
    COMMAND macdeployqt ${APP_PATH} -dmg
    COMMAND ${CMAKE_COMMAND} -E rename "${APP_NAME}.dmg" "${CMAKE_BINARY_DIR}/${APP_NAME}-${PROJECT_VERSION}.dmg"
  )
endif()
