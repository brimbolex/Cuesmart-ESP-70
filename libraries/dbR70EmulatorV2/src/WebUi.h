#ifndef WEB_UI_H
#define WEB_UI_H

#include <Arduino.h>
#include <WebServer.h>

#include "ConfigManager.h"

class EthernetServer;

namespace dbcan {

class WebUiRequest;
class WebUiWriter;

class WebUi {
 public:
  typedef void (*ActionHook)(uint8_t actionNumber, void* userData);

  explicit WebUi(ConfigManager& configManager);
  ~WebUi();

  WebUi(const WebUi&) = delete;
  WebUi& operator=(const WebUi&) = delete;

  bool begin(uint16_t port = 80);
  bool begin(EthernetServer& server);
  void handleClient();

  void setActionHook(ActionHook hook, void* userData = nullptr);
  uint8_t lastTriggeredAction() const;

 private:
  void registerHandlers();

  void handleRoot();
  void handleSaveConfig();
  void handleSaveConfig(const WebUiRequest& request);
  void handleClearErrors();
  void handleResetPeaks();
  void handleTriggerAction(uint8_t actionNumber);
  void handleRestart();
  void handleFactoryDefaults();
  void handleNotFound();

  void sendMainPage();
  void sendMainPage(WebUiWriter& writer);
  void redirectToRoot();
  void handleEthernetClient();

  ConfigManager& configManager_;
  WebServer* server_;
  EthernetServer* ethernetServer_;
  ActionHook actionHook_ = nullptr;
  void* actionHookUserData_ = nullptr;
  uint8_t lastTriggeredAction_ = 0;
};

}  // namespace dbcan

#endif  // WEB_UI_H
