#ifndef ARTI_CAN_INTERFACE_CAN_INTERFACE_H
#define ARTI_CAN_INTERFACE_CAN_INTERFACE_H

#include <arti_can_msgs/CanMessage.h>
#include <boost/core/noncopyable.hpp>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <vector>

namespace arti_can_interface
{
class CanInterface;

/**
 * Shared pointer to a CanInterface.
 */
using CanInterfacePtr = std::shared_ptr<CanInterface>;

/**
 * \brief Interface to the CAN network.
 *
 * Does not implement the actual sending -- this is done by a specific implementation.
 * Instances should be created using CanInterface::createCanInterface, and then accessed using the returned shared
 * pointer.
 */
class CanInterface : private boost::noncopyable
{
public:
  /**
   * \brief Definition of the callback function.
   *
   * The callback function should have the following signature:
   * \code[.cpp]
   * void myCallbackFunction(const arti_can_msgs::CanMessageConstPtr& can_message);
   * \endcode
   *
   * The CAN message contains the CAN id, message flags, and payload data.
   */
  using MessageCallback = std::function<void(const arti_can_msgs::CanMessageConstPtr&)>;

  virtual ~CanInterface() = default;

  /**
   * \brief Creates a CAN interface instance.
   *
   * Instantiates an implementation of the interface and returns a smart pointer to it.
   * \param device_name name of the CAN device to access.
   * \return a pointer to the created instance which can be shared and used for accessing the device
   */
  static CanInterfacePtr createCanInterface(const std::string& device_name);

  /**
   * \brief Sends a CAN message.
   *
   * This is the only function which needs to be implemented by a subclass for sending messages.
   * The function might block till sending is completed or till the message is enqueued for sending. In either case,
   * the function needs to ensure that successive calls preserve the order of the messages.
   *
   * \param can_message the message to send on the CAN interface, including all flags, the CAN id, and the data.
   */
  virtual void sendMessage(const arti_can_msgs::CanMessage& can_message) = 0;

  /**
   * \brief Sends a CAN message.
   *
   * For details, \see void sendMessage(const arti_can_msgs::CanMessage& can_message).
   *
   * \param can_message the message to send on the CAN interface, including all flags, the CAN id, and the data.
   */
  virtual void sendMessage(const arti_can_msgs::CanMessageConstPtr& can_message);

  /**
   * \brief Sends a CAN message with the given id and data.
   *
   * \param id id of the CAN message
   * \param data data to send
   */
  void sendMessage(uint32_t id, std::vector<uint8_t> data);

  /**
   * \brief Sends a CAN message with the given id, flags, and data.
   *
   * \param id id of the CAN message
   * \param rtr Remote Transmission Request (RTR) flag
   * \param extended_id Identifier Extension (IDE) flag
   * \param data data to send
   */
  void sendMessage(uint32_t id, bool rtr, bool extended_id, std::vector<uint8_t> data);

  /**
   * \brief Sends a CAN message with the given id, flags, and data.
   *
   * \param id id of the CAN message
   * \param rtr Remote Transmission Request (RTR) flag
   * \param extended_id Identifier Extension (IDE) flag
   * \param data buffer containing data to send
   * \param start_index index of first byte of data to send
   * \param size size of the block of data to send
   */
  void sendMessage(
    uint32_t id, bool rtr, bool extended_id, const std::vector<uint8_t>& data, size_t start_index, size_t size);

  /**
   * \brief Sends a CAN message with the given id, flags, and data.
   *
   * \param id id of the CAN message
   * \param rtr Remote Transmission Request (RTR) flag
   * \param extended_id Identifier Extension (IDE) flag
   * \param data pointer to data to send. Needs to be a properly allocated array of bytes with the defined size.
   * \param size size of the data block to send
   */
  void sendMessage(uint32_t id, bool rtr, bool extended_id, const uint8_t* data, size_t size);

  /**
   * \brief Registers a callback for the given CAN ids.
   *
   * The callback will be called in the read thread context.
   * Each callback with a CAN message blocks all other callbacks, till it is finished processing.
   * The callbacks and the received messages are handled in a thread-safe way.
   *
   * \param ids CAN ids to register the callback for
   * \param callback callback to call when a CAN message with one of the given ids is received, \see MessageCallback
   */
  void subscribe(const std::vector<uint32_t>& ids, const MessageCallback& callback);

  /**
   * \brief Registers a callback for the given CAN id.
   *
   * For details, \see void subscribe(const std::vector<uint32_t>& ids, const MessageCallback& callback)
   *
   * \param id CAN id to register the callback for
   * \param callback callback to call when a CAN message with the given id is received, \see MessageCallback
   */
  void subscribe(uint32_t id, MessageCallback callback);

  /**
   * \brief Registers a callback for all received CAN messages, regardless of their ids.
   *
   * The subscribed callback will be called in the read thread context.
   * Each callback with a CAN message blocks all other callbacks, till it is finished processing.
   * The callbacks and the received messages are handled in a thread-safe way.
   *
   * \param callback callback to call when a CAN message is received, \see MessageCallback
   */
  void subscribeToAll(MessageCallback callback);

  /**
   * \brief Creates a ROS CAN message with the given id and flags.
   *
   * \param id id of the CAN message
   * \param rtr Remote Transmission Request (RTR) flag
   * \param extended_id Identifier Extension (IDE) flag
   * \return a ROS CAN message initialized with the parameters above
   */
  static arti_can_msgs::CanMessagePtr createMessage(uint32_t id, bool rtr, bool extended_id);

  /**
   * \brief Checks whether the CAN message is valid concerning the id and flag combination.
   *
   * Subclasses might define specific conditions for valid CAN messages, e.g. if some of the flags are not supported by
   * the CAN device.
   *
   * \param can_message CAN message to check
   * \throw std::invalid_argument if the CAN message is invalid
   */
  virtual void checkMessage(const arti_can_msgs::CanMessage& can_message) = 0;

  /**
   * \brief Checks whether the CAN message is valid concerning the id and flag combination.
   *
   * For details, \see void checkMessage(const arti_can_msgs::CanMessage& can_message)
   *
   * \param can_message CAN message to check
   * \throw std::invalid_argument if the CAN message is invalid
   */
  void checkMessage(const arti_can_msgs::CanMessageConstPtr& can_message);

protected:
  /**
   * \brief Dispatches a CAN message to any callbacks registered for its CAN id.
   *
   * \param can_message CAN message to dispatch
   */
  void dispatchReceivedCanMessage(const arti_can_msgs::CanMessageConstPtr& can_message);

private:
  /**
   * \brief Calls the given callback if it is valid, and catches any exceptions it throws.
   *
   * Prints a warning if the callback is invalid or throws any exceptions.
   *
   * \param message_callback the callback to call
   * \param can_message the CAN message to pass to the callback
   */
  static void callMessageCallbackSafely(
    const MessageCallback& message_callback, const arti_can_msgs::CanMessageConstPtr& can_message);

  std::mutex callback_mutex_; ///< Mutex for thread-safe access to the callback collections.
  std::multimap<uint32_t, MessageCallback> callbacks_; ///< Registered callbacks by CAN id.
  std::vector<MessageCallback> callbacks_for_all_; ///< Callbacks registered for all CAN messages, regardless of id.
};
}

#endif //ARTI_CAN_INTERFACE_CAN_INTERFACE_H
