#ifndef BAGSUBSCRIBER_HPP
#define BAGSUBSCRIBER_HPP
/*
 * bagSubscriber.hpp
 *
 *  Created on: Nov 29, 2012
 *      Author: kabamaru
 */


/**
* Inherits from message_filters::SimpleFilter<M> to use protected signalMessage function
* A dummy class bases on SimpleFilter<M>. It's being fed with the messages read from the bag.
*/
template <class M>
class bagSubscriber : public message_filters::SimpleFilter<M>
{
   public:
     bagSubscriber() {}
     /**
      * It sends a messages to the subscriber
      * @param msg The message variable
      */
     void newMessage(const boost::shared_ptr<M const> &msg)
     {
       signalMessage(msg);
     }
};

#endif
