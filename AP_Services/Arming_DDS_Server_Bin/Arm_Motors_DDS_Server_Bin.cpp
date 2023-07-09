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
static uxrObjectId replier_id;

bool isArmable = true;

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

    bool arm;
    ucdr_deserialize_bool(ub,&arm);

    if (arm){
        printf("Request recieved for Arming \n");
    }else{
        printf("Request recieved for Disarming \n");
    }
    uint8_t reply_buffer[8] = {
        0
    };
    
    ucdrBuffer reply_ub;
    
    bool result = false;
    if(isArmable){
        result = arm;
    }else{
        result = false;
    }
    
    ucdr_init_buffer(&reply_ub, reply_buffer, sizeof(reply_buffer));
    ucdr_serialize_bool(&reply_ub,result);

    uxr_buffer_reply(session, reliable_out, replier_id, sample_id, reply_buffer, sizeof(reply_buffer));

    if(result){
        printf("Reply : Armed \n");
    }else{
        printf("Reply : Disarmed \n");
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
    isArmable = (args == 4) ? atoi(argv[3]) : 1;
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
    participant_id = uxr_object_id(0x01, UXR_PARTICIPANT_ID);
    uint16_t participant_req = uxr_buffer_create_participant_bin(&session,reliable_out,participant_id,0,"ArmMotors_Server",UXR_REPLACE);
    uxrQoS_t qos = {
        .durability = UXR_DURABILITY_TRANSIENT_LOCAL,
        .reliability = UXR_RELIABILITY_RELIABLE,
        .history = UXR_HISTORY_KEEP_ALL,
        .depth = 1
    };
    
    replier_id = uxr_object_id(0x01, UXR_REPLIER_ID);
    uint16_t replier_req = uxr_buffer_create_replier_bin(&session,reliable_out,replier_id,participant_id,"ArmMotorsService","ArmMotors_Request","ArmMotors_Response","ArmMotorsService_Request","ArmMotorsService_Reply",qos,UXR_REPLACE);
    
    // Send create entities message and wait its status
    uint8_t status[2];
    uint16_t requests[2] = {
        participant_req, replier_req
    };
    if (!uxr_run_session_until_all_status(&session, 1000, requests, status, 2))
    {
        printf("Error at create entities: participant: %i requester: %i\n", status[0], status[1]);
        return 1;
    }

    printf("Entities Created Successfully \n");

    // Request  requests
    uxrDeliveryControl delivery_control = {
        0
    };
    delivery_control.max_samples = UXR_MAX_SAMPLES_UNLIMITED;
    uint16_t read_data_req =
            uxr_buffer_request_data(&session, reliable_out, replier_id, reliable_in, &delivery_control);
    (void) read_data_req;

    // Read request
    bool connected = true;
    while (connected)
    {
        connected = uxr_run_session_time(&session, 1000);
    }

    return 0;
}