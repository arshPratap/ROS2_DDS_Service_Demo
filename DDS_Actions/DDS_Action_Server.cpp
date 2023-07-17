#include <uxr/client/client.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>

#define STREAM_HISTORY  8
#define BUFFER_SIZE     UXR_CONFIG_UDP_TRANSPORT_MTU* STREAM_HISTORY

static uxrStreamId reliable_out;
static uxrStreamId reliable_in;

static uxrObjectId participant_id;
static uxrObjectId goal_replier_id;
//static uxrObjectId res_replier_id;
static uxrObjectId feedback_topic_id;
static uxrObjectId feedback_pub_id;
static uxrObjectId feedback_dw_id;

bool connected = true;
bool action_start = false;
int32_t  count_order = 0;
int32_t  sample_count = 0;
int32_t  GM_seq = 1; 

uint32_t size_of_feedback(const int32_t* GM_Seq,uint32_t size){
    uint32_t previous_size = size;
    size += (uint32_t)(ucdr_alignment(size, 4) + 4);
    return size-previous_size;
}
void on_request(
        uxrSession* session,
        uxrObjectId object_id,
        uint16_t request_id,
        SampleIdentity* sample_id,
        ucdrBuffer* ub,
        uint16_t length,
        void* args)
{
    (void) object_id;
    (void) request_id;
    (void) length;
    (void) args;

    if(!action_start){
        if(ucdr_deserialize_int32_t(ub,&count_order)){
            action_start = true;
            printf("Action : Goal Request Recieved and Action Started \n");
            // send data
            while(connected && sample_count < count_order){
                uint32_t topic_size = size_of_feedback(&GM_seq,0);
                uxr_prepare_output_stream(session,reliable_out,feedback_dw_id,ub,topic_size); 
                if(ucdr_serialize_int32_t(ub,GM_seq)){
                    printf("Action : Feedback Sending Data : %d \n",GM_seq);
                    sample_count++;
                    GM_seq = 2*GM_seq;
                }else{
                    printf("Action : Feedback cannot be sent \n");
                }
            }

            // send result
            uint8_t reply_buffer[8] = {
                0
            };
            ucdrBuffer reply_ub;
            ucdr_init_buffer(&reply_ub, reply_buffer, sizeof(reply_buffer));
            ucdr_serialize_int32_t(&reply_ub,GM_seq);
            bool res = uxr_buffer_reply(session,reliable_out,goal_replier_id,sample_id,reply_buffer,sizeof(reply_buffer));
            if(res){
                printf("Action : Final Result Sent %d \n",GM_seq);
                GM_seq = 1;
                action_start = false;
                count_order = 0;
                sample_count = 0;
            }else{
                printf("Action : Final Result not sent \n");
            }
        }else{
            printf("Action : Goal Request Recieved and Action can't be Started \n");
        }
    }
}

int main(
        int args,
        char** argv)
{
    if (3 > args || 0 == atoi(argv[2]))
    {
        printf("usage: program [-h | --help] | ip port [armable]\n");
        printf("usage: armable: 0 to set arming condition to false \n");
        printf("usage: armable: 1 to set arming condition to true \n");
        return 0;
    }

    char* ip = argv[1];
    char* port = argv[2];
    uint32_t key = 0xCCCCDDDD;
    
    // Transport
    uxrUDPTransport transport;
    if (!uxr_init_udp_transport(&transport, UXR_IPv4, ip, port))
    {
        printf("Error at init transport.\n");
        return 1;
    }

    // Session
    uxrSession session;
    uxr_init_session(&session, &transport.comm, key);
    uxr_set_request_callback(&session, on_request, 0);
    if (!uxr_create_session(&session))
    {
        printf("Error at init session.\n");
        return 1;
    }

    // Streams
    uint8_t output_reliable_stream_buffer[BUFFER_SIZE];
    reliable_out = uxr_create_output_reliable_stream(&session, output_reliable_stream_buffer, BUFFER_SIZE,
                    STREAM_HISTORY);

    uint8_t input_reliable_stream_buffer[BUFFER_SIZE];
    reliable_in = uxr_create_input_reliable_stream(&session, input_reliable_stream_buffer, BUFFER_SIZE, STREAM_HISTORY);

    // Create entities
    //participant
    participant_id = uxr_object_id(0x01, UXR_PARTICIPANT_ID);
    const char* participant_xml = "<dds>"
            "<participant>"
            "<rtps>"
            "<name>DDS_Action_GM_Server</name>"
            "</rtps>"
            "</participant>"
            "</dds>";
    uint16_t participant_req = uxr_buffer_create_participant_xml(&session, reliable_out, participant_id, 0,
                    participant_xml, UXR_REPLACE);

    //goal-replier
    goal_replier_id = uxr_object_id(0x01, UXR_REPLIER_ID);
    const char* goal_replier_xml = "<dds>"
            "<replier profile_name=\"GM_GoalReplier\""
            "service_name=\"GM_GoalService\""
            "request_type=\"GM_Goal\""
            "reply_type=\"GM_Result\">"
            "</replier>"
            "</dds>";
    uint16_t goal_replier_req = uxr_buffer_create_replier_xml(&session, reliable_out, goal_replier_id, participant_id,
                    goal_replier_xml, UXR_REPLACE);

    //feedback-topic
    feedback_topic_id = uxr_object_id(0x01, UXR_TOPIC_ID);
    const char* feedback_topic_xml = "<dds>"
            "<topic>"
            "<name>GM_FeedbackTopic</name>"
            "<dataType>GM_Feedback</dataType>"
            "</topic>"
            "</dds>";
    uint16_t feedback_topic_req = uxr_buffer_create_topic_xml(&session,reliable_out,feedback_topic_id,participant_id,feedback_topic_xml,UXR_REPLACE);

    //feedback-pub
    feedback_pub_id = uxr_object_id(0x01, UXR_PUBLISHER_ID);
    const char* feedback_pub_xml = "";
    uint16_t feedback_pub_req = uxr_buffer_create_publisher_xml(&session,reliable_out,feedback_pub_id,participant_id,feedback_pub_xml,UXR_REPLACE);
    
    //feedback-dw
    feedback_dw_id = uxr_object_id(0x01,UXR_TOPIC_ID);
    const char* feedback_dw_xml = "<dds>"
            "<data_writer>"
            "<topic>"
            "<kind>NO_KEY</kind>"
            "<name>GM_FeedbackTopic</name>"
            "<dataType>GM_Feedback</dataType>"
            "</topic>"
            "</data_writer>"
            "</dds>";
    uint16_t feedback_dw_req = uxr_buffer_create_datawriter_xml(&session,reliable_out,feedback_dw_id,feedback_pub_id,feedback_dw_xml,UXR_REPLACE);
    
    // Send create entities message and wait its status
    uint8_t status[5];
    uint16_t requests[5] = {
        participant_req,
        goal_replier_req,
        feedback_topic_req,
        feedback_pub_req,
        feedback_dw_req,
    };
    if (!uxr_run_session_until_all_status(&session, 1000, requests, status, 5))
    {
        printf("Error at create entities \n");
        printf("Participant : %i \n",status[0]);
        printf("Goal Replier : %i \n",status[1]);
        printf("Feedback Topic : %i \n",status[2]);
        printf("Feedback Publisher : %i \n",status[3]);
        printf("Feedback Datawriter : %i \n",status[4]);
        return 1;
    }

    printf("Entities Created Successfully \n");

    // Request  requests
    uxrDeliveryControl delivery_control = {
        0
    };
    delivery_control.max_samples = UXR_MAX_SAMPLES_UNLIMITED;
    
    uxr_buffer_request_data(&session, reliable_out, goal_replier_id, reliable_in, &delivery_control);
    //uxr_buffer_request_data(&session, reliable_out, res_replier_id, reliable_in, &delivery_control);
    

    while (connected)
    {
        connected = uxr_run_session_time(&session, 1000);
    }

    return 0;
}