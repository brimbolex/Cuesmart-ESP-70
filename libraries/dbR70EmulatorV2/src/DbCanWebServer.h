#ifndef DB_CAN_WEB_SERVER_H
#define DB_CAN_WEB_SERVER_H

#include <Arduino.h>

#include "ConfigManager.h"
#include "DbCanActions.h"

class EthernetServer;

namespace dbcan {

class DbCanWebServer {
 public:
  DbCanWebServer(ConfigManager& configManager, ActionQueue& actionQueue, EthernetServer& server);
  ~DbCanWebServer();

  DbCanWebServer(const DbCanWebServer&) = delete;
  DbCanWebServer& operator=(const DbCanWebServer&) = delete;

  bool begin();
  void loop();

  size_t buildStatusJson(char* out, size_t capacity) const;
  bool handleConfigUpdate(const char* body, size_t length);
  bool handleActionTrigger(const char* body, size_t length);
  bool handleServiceCommand(const char* body, size_t length);

  const char* lastError() const;

 private:
  struct RequestState;

  void resetRequest();
  void acceptClient();
  void readClientData();
  void dispatchRequest();

  void sendHtmlPage();
  void sendStatusJson();
  void sendJsonResult(int code, bool ok, const char* message = nullptr);
  void sendTextResult(int code, const char* text);
  void sendResponseHeader(int code, const char* phrase, const char* contentType, size_t contentLength);
  void closeClient();

  bool parseRequestHead();
  bool bodyComplete() const;
  void setError(const char* message);

  ConfigManager& configManager_;
  ActionQueue& actionQueue_;
  EthernetServer& server_;
  RequestState* request_;
  char lastError_[64];
};

}  // namespace dbcan

#endif  // DB_CAN_WEB_SERVER_H
