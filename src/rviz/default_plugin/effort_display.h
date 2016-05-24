#ifndef EFFORT_DISPLAY_H
#define EFFORT_DISPLAY_H

#ifndef Q_MOC_RUN
#include <boost/circular_buffer.hpp>
#endif

#include <sensor_msgs/JointState.h>
#include <rviz/message_filter_display.h>

namespace Ogre
{
    class SceneNode;
}

namespace rviz
{
    class FloatProperty;
    class IntProperty;
    class StringProperty;
    class CategoryProperty;
    class BoolProperty;
    class RosTopicProperty;
}

namespace urdf
{
    class Model;
}

// MessageFilter to support message with out frame_id
// https://code.ros.org/trac/ros-pkg/ticket/5467
namespace tf
{
#ifdef TF_MESSAGEFILTER_DEBUG
# undef TF_MESSAGEFILTER_DEBUG
#endif
#define TF_MESSAGEFILTER_DEBUG(fmt, ...) \
  ROS_DEBUG_NAMED("message_filter", "MessageFilter [target=%s]: " fmt, getTargetFramesString().c_str(), __VA_ARGS__)

#ifdef TF_MESSAGEFILTER_WARN
# undef TF_MESSAGEFILTER_WARN
#endif
#define TF_MESSAGEFILTER_WARN(fmt, ...) \
  ROS_WARN_NAMED("message_filter", "MessageFilter [target=%s]: " fmt, getTargetFramesString().c_str(), __VA_ARGS__)

    class MessageFilterJointState : public MessageFilter<sensor_msgs::JointState>
    {
	typedef sensor_msgs::JointState M;
public:
	typedef boost::shared_ptr<M const> MConstPtr;
	typedef ros::MessageEvent<M const> MEvent;
	typedef boost::function<void(const MConstPtr&, FilterFailureReason)> FailureCallback;
#ifdef RVIZ_USE_BOOST_SIGNAL_1
	typedef boost::signal<void(const MConstPtr&, FilterFailureReason)> FailureSignal;
#else
	typedef boost::signals2::signal<void(const MConstPtr&, FilterFailureReason)> FailureSignal;
#endif

	// If you hit this assert your message does not have a header, or does not have the HasHeader trait defined for it
	ROS_STATIC_ASSERT(ros::message_traits::HasHeader<M>::value);

	/**
	 * \brief Constructor
	 *
	 * \param tf The tf::Transformer this filter should use
	 * \param target_frame The frame this filter should attempt to transform to.  To use multiple frames, pass an empty string here and use the setTargetFrames() function.
	 * \param queue_size The number of messages to queue up before throwing away old ones.  0 means infinite (dangerous).
	 * \param nh The NodeHandle to use for any necessary operations
	 * \param max_rate The maximum rate to check for newly transformable messages
	 */
	MessageFilterJointState(Transformer& tf, const std::string& target_frame, uint32_t queue_size, ros::NodeHandle nh = ros::NodeHandle(), ros::Duration max_rate = ros::Duration(0.01))
	    : MessageFilter<sensor_msgs::JointState>(tf, target_frame, queue_size, nh, max_rate)
            , tf_(tf)
	    , nh_(nh)
	    , max_rate_(max_rate)
	    , queue_size_(queue_size)
	    {
		init();

		setTargetFrame(target_frame);
	    }

	/**
	 * \brief Constructor
	 *
	 * \param f The filter to connect this filter's input to.  Often will be a message_filters::Subscriber.
	 * \param tf The tf::Transformer this filter should use
	 * \param target_frame The frame this filter should attempt to transform to.  To use multiple frames, pass an empty string here and use the setTargetFrames() function.
	 * \param queue_size The number of messages to queue up before throwing away old ones.  0 means infinite (dangerous).
	 * \param nh The NodeHandle to use for any necessary operations
	 * \param max_rate The maximum rate to check for newly transformable messages
	 */
	template<class F>
	MessageFilterJointState(F& f, Transformer& tf, const std::string& target_frame, uint32_t queue_size, ros::NodeHandle nh = ros::NodeHandle(), ros::Duration max_rate = ros::Duration(0.01))
	    : tf_(tf)
	    , nh_(nh)
	    , max_rate_(max_rate)
	    , queue_size_(queue_size)
            , MessageFilter<sensor_msgs::JointState>(f, tf, target_frame, queue_size, nh, max_rate)
	    {
		init();

		setTargetFrame(target_frame);

		connectInput(f);
	    }

	/**
	 * \brief Connect this filter's input to another filter's output.  If this filter is already connected, disconnects first.
	 */
	template<class F>
	void connectInput(F& f)
	    {
		message_connection_.disconnect();
		message_connection_ = f.registerCallback(&MessageFilterJointState::incomingMessage, this);
	    }

	/**
	 * \brief Destructor
	 */
	~MessageFilterJointState()
	    {
		message_connection_.disconnect();
		tf_.removeTransformsChangedListener(tf_connection_);

		clear();

		TF_MESSAGEFILTER_DEBUG("Successful Transforms: %llu, Failed Transforms: %llu, Discarded due to age: %llu, Transform messages received: %llu, Messages received: %llu, Total dropped: %llu",
				       (long long unsigned int)successful_transform_count_, (long long unsigned int)failed_transform_count_,
				       (long long unsigned int)failed_out_the_back_count_, (long long unsigned int)transform_message_count_,
				       (long long unsigned int)incoming_message_count_, (long long unsigned int)dropped_message_count_);

	    }

	/**
	 * \brief Set the frame you need to be able to transform to before getting a message callback
	 */
	void setTargetFrame(const std::string& target_frame)
	    {
		std::vector<std::string> frames;
		frames.push_back(target_frame);
		setTargetFrames(frames);
	    }

	/**
	 * \brief Set the frames you need to be able to transform to before getting a message callback
	 */
	void setTargetFrames(const std::vector<std::string>& target_frames)
	    {
		boost::mutex::scoped_lock list_lock(messages_mutex_);
		boost::mutex::scoped_lock string_lock(target_frames_string_mutex_);

		target_frames_ = target_frames;

		std::stringstream ss;
		for (std::vector<std::string>::iterator it = target_frames_.begin(); it != target_frames_.end(); ++it)
		{
		    *it = tf::resolve(tf_.getTFPrefix(), *it);
		    ss << *it << " ";
		}
		target_frames_string_ = ss.str();
	    }
	/**
	 * \brief Get the target frames as a string for debugging
	 */
	std::string getTargetFramesString()
	    {
		boost::mutex::scoped_lock lock(target_frames_string_mutex_);
		return target_frames_string_;
	    };

	/**
	 * \brief Set the required tolerance for the notifier to return true
	 */
	void setTolerance(const ros::Duration& tolerance)
	    {
		time_tolerance_ = tolerance;
	    }

	/**
	 * \brief Clear any messages currently in the queue
	 */
	void clear()
	    {
		boost::mutex::scoped_lock lock(messages_mutex_);

		TF_MESSAGEFILTER_DEBUG("%s", "Cleared");

		messages_.clear();
		message_count_ = 0;

		warned_about_unresolved_name_ = false;
		warned_about_empty_frame_id_ = false;
	    }

	void add(const MEvent& evt)
	    {
		boost::mutex::scoped_lock lock(messages_mutex_);

		testMessages();

		if (!testMessage(evt))
		{
		    // If this message is about to push us past our queue size, erase the oldest message
		    if (queue_size_ != 0 && message_count_ + 1 > queue_size_)
		    {
			++dropped_message_count_;
			const MEvent& front = messages_.front();
			TF_MESSAGEFILTER_DEBUG("Removed oldest message because buffer is full, count now %d (frame_id=%s, stamp=%f)", message_count_, front.getMessage()->header.frame_id.c_str(), front.getMessage()->header.stamp.toSec());
			signalFailure(messages_.front(), filter_failure_reasons::Unknown);

			messages_.pop_front();
			--message_count_;
		    }

		    // Add the message to our list
		    messages_.push_back(evt);
		    ++message_count_;
		}

		TF_MESSAGEFILTER_DEBUG("Added message in frame %s at time %.3f, count now %d", evt.getMessage()->header.frame_id.c_str(), evt.getMessage()->header.stamp.toSec(), message_count_);

		++incoming_message_count_;
	    }

	/**
	 * \brief Manually add a message into this filter.
	 * \note If the message (or any other messages in the queue) are immediately transformable this will immediately call through to the output callback, possibly
	 * multiple times
	 */
	void add(const MConstPtr& message)
	    {
		boost::shared_ptr<std::map<std::string, std::string> > header(new std::map<std::string, std::string>);
		(*header)["callerid"] = "unknown";
		add(MEvent(message, header, ros::Time::now()));
	    }

	/**
	 * \brief Register a callback to be called when a message is about to be dropped
	 * \param callback The callback to call
	 */
	message_filters::Connection registerFailureCallback(const FailureCallback& callback)
	    {
		boost::mutex::scoped_lock lock(failure_signal_mutex_);
		return message_filters::Connection(boost::bind(&MessageFilterJointState::disconnectFailure, this, _1), failure_signal_.connect(callback));
	    }

	virtual void setQueueSize( uint32_t new_queue_size )
	    {
		queue_size_ = new_queue_size;
	    }

	virtual uint32_t getQueueSize()
	    {
		return queue_size_;
	    }

    private:

	void init()
	    {
		message_count_ = 0;
		new_transforms_ = false;
		successful_transform_count_ = 0;
		failed_transform_count_ = 0;
		failed_out_the_back_count_ = 0;
		transform_message_count_ = 0;
		incoming_message_count_ = 0;
		dropped_message_count_ = 0;
		time_tolerance_ = ros::Duration(0.0);
		warned_about_unresolved_name_ = false;
		warned_about_empty_frame_id_ = false;

		tf_connection_ = tf_.addTransformsChangedListener(boost::bind(&MessageFilterJointState::transformsChanged, this));

		max_rate_timer_ = nh_.createTimer(max_rate_, &MessageFilterJointState::maxRateTimerCallback, this);
	    }

	typedef std::list<MEvent> L_Event;

	bool testMessage(const MEvent& evt)
	    {
		const MConstPtr& message = evt.getMessage();
		std::string callerid = evt.getPublisherName();
		std::string frame_id = ros::message_traits::FrameId<M>::value(*message);
		ros::Time stamp = ros::message_traits::TimeStamp<M>::value(*message);

		// FIXED
		if (frame_id.empty()) frame_id = * (target_frames_.begin());
		//Throw out messages which have an empty frame_id
		if (frame_id.empty())
		{
		    if (!warned_about_empty_frame_id_)
		    {
			warned_about_empty_frame_id_ = true;
			TF_MESSAGEFILTER_WARN("Discarding message from [%s] due to empty frame_id.  This message will only print once.", callerid.c_str());
		    }
		    signalFailure(evt, filter_failure_reasons::EmptyFrameID);
		    return true;
		}

		if (frame_id[0] != '/')
		{
		    std::string unresolved = frame_id;
		    frame_id = tf::resolve(tf_.getTFPrefix(), frame_id);

		    if (!warned_about_unresolved_name_)
		    {
			warned_about_unresolved_name_ = true;
			ROS_WARN("Message from [%s] has a non-fully-qualified frame_id [%s]. Resolved locally to [%s].  This is will likely not work in multi-robot systems.  This message will only print once.", callerid.c_str(), unresolved.c_str(), frame_id.c_str());
		    }
		}

		//Throw out messages which are too old
		//! \todo combine getLatestCommonTime call with the canTransform call
		for (std::vector<std::string>::iterator target_it = target_frames_.begin(); target_it != target_frames_.end(); ++target_it)
		{
		    const std::string& target_frame = *target_it;

		    if (target_frame != frame_id && stamp != ros::Time(0))
		    {
			ros::Time latest_transform_time ;

			tf_.getLatestCommonTime(frame_id, target_frame, latest_transform_time, 0) ;
			if (stamp + tf_.getCacheLength() < latest_transform_time)
			{
			    ++failed_out_the_back_count_;
			    ++dropped_message_count_;
			    TF_MESSAGEFILTER_DEBUG("Discarding Message, in frame %s, Out of the back of Cache Time(stamp: %.3f + cache_length: %.3f < latest_transform_time %.3f.  Message Count now: %d", message->header.frame_id.c_str(), message->header.stamp.toSec(),  tf_.getCacheLength().toSec(), latest_transform_time.toSec(), message_count_);

			    last_out_the_back_stamp_ = stamp;
			    last_out_the_back_frame_ = frame_id;

			    signalFailure(evt, filter_failure_reasons::OutTheBack);
			    return true;
			}
		    }

		}

		bool ready = !target_frames_.empty();
		for (std::vector<std::string>::iterator target_it = target_frames_.begin(); ready && target_it != target_frames_.end(); ++target_it)
		{
		    std::string& target_frame = *target_it;
		    if (time_tolerance_ != ros::Duration(0.0))
		    {
			ready = ready && (tf_.canTransform(target_frame, frame_id, stamp) &&
					  tf_.canTransform(target_frame, frame_id, stamp + time_tolerance_) );
		    }
		    else
		    {
			ready = ready && tf_.canTransform(target_frame, frame_id, stamp);
		    }
		}

		if (ready)
		{
		    TF_MESSAGEFILTER_DEBUG("Message ready in frame %s at time %.3f, count now %d", frame_id.c_str(), stamp.toSec(), message_count_);

		    ++successful_transform_count_;

		    signalMessage(evt);
		}
		else
		{
		    ++failed_transform_count_;
		}

		return ready;
	    }

	void testMessages()
	    {
		if (!messages_.empty() && getTargetFramesString() == " ")
		{
		    ROS_WARN_NAMED("message_notifier", "MessageFilter [target=%s]: empty target frame", getTargetFramesString().c_str());
		}

		int i = 0;

		L_Event::iterator it = messages_.begin();
		for (; it != messages_.end(); ++i)
		{
		    MEvent& evt = *it;

		    if (testMessage(evt))
		    {
			--message_count_;
			it = messages_.erase(it);
		    }
		    else
		    {
			++it;
		    }
		}
	    }

	void maxRateTimerCallback(const ros::TimerEvent&)
	    {
		boost::mutex::scoped_lock list_lock(messages_mutex_);
		if (new_transforms_)
		{
		    testMessages();
		    new_transforms_ = false;
		}

		checkFailures();
	    }

	/**
	 * \brief Callback that happens when we receive a message on the message topic
	 */
	void incomingMessage(const ros::MessageEvent<M const>& evt)
	    {
		add(evt);
	    }

	void transformsChanged()
	    {
		new_transforms_ = true;

		++transform_message_count_;
	    }

	void checkFailures()
	    {
		if (next_failure_warning_.isZero())
		{
		    next_failure_warning_ = ros::Time::now() + ros::Duration(15);
		}

		if (ros::Time::now() >= next_failure_warning_)
		{
		    if (incoming_message_count_ - message_count_ == 0)
		    {
			return;
		    }

		    double dropped_pct = (double)dropped_message_count_ / (double)(incoming_message_count_ - message_count_);
		    if (dropped_pct > 0.95)
		    {
			TF_MESSAGEFILTER_WARN("Dropped %.2f%% of messages so far. Please turn the [%s.message_notifier] rosconsole logger to DEBUG for more information.", dropped_pct*100, ROSCONSOLE_DEFAULT_NAME);
			next_failure_warning_ = ros::Time::now() + ros::Duration(60);

			if ((double)failed_out_the_back_count_ / (double)dropped_message_count_ > 0.5)
			{
			    TF_MESSAGEFILTER_WARN("  The majority of dropped messages were due to messages growing older than the TF cache time.  The last message's timestamp was: %f, and the last frame_id was: %s", last_out_the_back_stamp_.toSec(), last_out_the_back_frame_.c_str());
			}
		    }
		}
	    }

	void disconnectFailure(const message_filters::Connection& c)
	    {
		boost::mutex::scoped_lock lock(failure_signal_mutex_);
		c.getBoostConnection().disconnect();
	    }

	void signalFailure(const MEvent& evt, FilterFailureReason reason)
	    {
		boost::mutex::scoped_lock lock(failure_signal_mutex_);
		failure_signal_(evt.getMessage(), reason);
	    }

	Transformer& tf_; ///< The Transformer used to determine if transformation data is available
	ros::NodeHandle nh_; ///< The node used to subscribe to the topic
	ros::Duration max_rate_;
	ros::Timer max_rate_timer_;
	std::vector<std::string> target_frames_; ///< The frames we need to be able to transform to before a message is ready
	std::string target_frames_string_;
	boost::mutex target_frames_string_mutex_;
	uint32_t queue_size_; ///< The maximum number of messages we queue up

	L_Event messages_; ///< The message list
	uint32_t message_count_; ///< The number of messages in the list.  Used because messages_.size() has linear cost
	boost::mutex messages_mutex_; ///< The mutex used for locking message list operations

	bool new_messages_; ///< Used to skip waiting on new_data_ if new messages have come in while calling back
	volatile bool new_transforms_; ///< Used to skip waiting on new_data_ if new transforms have come in while calling back or transforming data

	bool warned_about_unresolved_name_;
	bool warned_about_empty_frame_id_;

	uint64_t successful_transform_count_;
	uint64_t failed_transform_count_;
	uint64_t failed_out_the_back_count_;
	uint64_t transform_message_count_;
	uint64_t incoming_message_count_;
	uint64_t dropped_message_count_;

	ros::Time last_out_the_back_stamp_;
	std::string last_out_the_back_frame_;

	ros::Time next_failure_warning_;

	ros::Duration time_tolerance_; ///< Provide additional tolerance on time for messages which are stamped but can have associated duration

#ifdef RVIZ_USE_BOOST_SIGNAL_1
	boost::signals::connection tf_connection_;
#else
	boost::signals2::connection tf_connection_;
#endif
	message_filters::Connection message_connection_;

	FailureSignal failure_signal_;
	boost::mutex failure_signal_mutex_;
    };
}

#ifndef Q_MOC_RUN
#include <message_filters/subscriber.h>
#include <tf/message_filter.h>
#endif

#include "rviz/display_context.h"
#include "rviz/frame_manager.h"
#include "rviz/properties/ros_topic_property.h"

namespace rviz
{
/** @brief Display subclass using a tf::MessageFilter, templated on the ROS message type.
 *
 * This class brings together some common things used in many Display
 * types.  It has a tf::MessageFilter to filter incoming messages, and
 * it handles subscribing and unsubscribing when the display is
 * enabled or disabled.  It also has an Ogre::SceneNode which  */
class MessageFilterJointStateDisplay: public _RosTopicDisplay
{
// No Q_OBJECT macro here, moc does not support Q_OBJECT in a templated class.
public:
  /** @brief Convenience typedef so subclasses don't have to use
   * the long templated class name to refer to their super class. */
  typedef MessageFilterJointStateDisplay MFDClass;

  MessageFilterJointStateDisplay()
    : tf_filter_( NULL )
    , messages_received_( 0 )
    {
      QString message_type = QString::fromStdString( ros::message_traits::datatype<sensor_msgs::JointState>() );
      topic_property_->setMessageType( message_type );
      topic_property_->setDescription( message_type + " topic to subscribe to." );
    }

  virtual void onInitialize()
    {
      tf_filter_ = new tf::MessageFilterJointState( *context_->getTFClient(),
                                                    fixed_frame_.toStdString(), 10, update_nh_ );

      tf_filter_->connectInput( sub_ );
      tf_filter_->registerCallback( boost::bind( &MessageFilterJointStateDisplay::incomingMessage, this, _1 ));
      context_->getFrameManager()->registerFilterForTransformStatusCheck( tf_filter_, this );
    }

  virtual ~MessageFilterJointStateDisplay()
    {
      unsubscribe();
      delete tf_filter_;
    }

  virtual void reset()
    {
      Display::reset();
      tf_filter_->clear();
      messages_received_ = 0;
    }

protected:
  virtual void updateTopic()
    {
      unsubscribe();
      reset();
      subscribe();
      context_->queueRender();
    }

  virtual void subscribe()
    {
      if( !isEnabled() )
      {
        return;
      }

      try
      {
        sub_.subscribe( update_nh_, topic_property_->getTopicStd(), 10 );
        setStatus( StatusProperty::Ok, "Topic", "OK" );
      }
      catch( ros::Exception& e )
      {
        setStatus( StatusProperty::Error, "Topic", QString( "Error subscribing: " ) + e.what() );
      }
    }

  virtual void unsubscribe()
    {
      sub_.unsubscribe();
    }

  virtual void onEnable()
    {
      subscribe();
    }

  virtual void onDisable()
    {
      unsubscribe();
      reset();
    }

  virtual void fixedFrameChanged()
    {
      tf_filter_->setTargetFrame( fixed_frame_.toStdString() );
      reset();
    }

  /** @brief Incoming message callback.  Checks if the message pointer
   * is valid, increments messages_received_, then calls
   * processMessage(). */
  void incomingMessage( const sensor_msgs::JointState::ConstPtr& msg )
    {
      if( !msg )
      {
        return;
      }

      ++messages_received_;
      setStatus( StatusProperty::Ok, "Topic", QString::number( messages_received_ ) + " messages received" );

      processMessage( msg );
    }

  /** @brief Implement this to process the contents of a message.
   *
   * This is called by incomingMessage(). */
  virtual void processMessage( const sensor_msgs::JointState::ConstPtr& msg ) = 0;

  message_filters::Subscriber<sensor_msgs::JointState> sub_;
  tf::MessageFilterJointState* tf_filter_;
  uint32_t messages_received_;
};
} // rviz

namespace rviz
{
    class JointInfo: public QObject {
        Q_OBJECT
        public:
        JointInfo(const std::string name, rviz::Property* parent_category);
        ~JointInfo();

        void setEffort(double e);
        double getEffort();
        void setMaxEffort(double m);
        double getMaxEffort();
        bool getEnabled() const;

        ros::Time last_update_;

    public Q_SLOTS:
        void updateVisibility();

    private:
        std::string name_;
        double effort_, max_effort_;

        rviz::Property* category_;
        rviz::FloatProperty* effort_property_;
        rviz::FloatProperty* max_effort_property_;
    };

    typedef std::set<JointInfo*> S_JointInfo;
    typedef std::vector<std::string> V_string;

    class EffortVisual;

    class EffortDisplay: public rviz::MessageFilterJointStateDisplay
    {
    Q_OBJECT
    public:
	// Constructor.  pluginlib::ClassLoader creates instances by calling
	// the default constructor, so make sure you have one.
	EffortDisplay();
	virtual ~EffortDisplay();

	// Overrides of public virtual functions from the Display class.
	virtual void onInitialize();
	virtual void reset();

    private Q_SLOTS:
	// Helper function to apply color and alpha to all visuals.
	void updateColorAndAlpha();
        void updateHistoryLength();
        void updateRobotDescription();

        JointInfo* getJointInfo( const std::string& joint);
        JointInfo* createJoint(const std::string &joint);

    protected:
        // overrides from Display
        virtual void onEnable();
        virtual void onDisable();

        // load
        void load();
        void clear();

	// The object for urdf model
	boost::shared_ptr<urdf::Model> robot_model_;

        //
        std::string robot_description_;

    private:
	void processMessage( const sensor_msgs::JointState::ConstPtr& msg );

        // Storage for the list of visuals.  It is a circular buffer where
        // data gets popped from the front (oldest) and pushed to the back (newest)
        boost::circular_buffer<boost::shared_ptr<EffortVisual> > visuals_;

        typedef std::map<std::string, JointInfo*> M_JointInfo;
        M_JointInfo joints_;

	// Property objects for user-editable properties.
        rviz::FloatProperty *alpha_property_,* width_property_,* scale_property_;
	rviz::IntProperty *history_length_property_;

        rviz::StringProperty *robot_description_property_;
        rviz::Property *joints_category_;
        rviz::BoolProperty *all_enabled_property_;
    };
} // end namespace rviz_plugin_tutorials

#endif // EFFORT_DISPLAY_H
