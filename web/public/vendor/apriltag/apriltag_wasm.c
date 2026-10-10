#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <emscripten.h>
#include "apriltag.h"
#include "tag36h11.h"
#include "tag25h9.h"
#include "tag16h5.h"
#include "tagCircle21h7.h"
#include "tagCircle49h12.h"
#include "tagCustom48h12.h"
#include "tagStandard41h12.h"
#include "tagStandard52h13.h"
#include "common/image_u8.h"

// Global variables to hold detector and tag family instances
apriltag_detector_t *td = NULL;
apriltag_family_t *tf_36h11 = NULL;
apriltag_family_t *tf_25h9 = NULL;
apriltag_family_t *tf_16h5 = NULL;
apriltag_family_t *tf_circle21h7 = NULL;
apriltag_family_t *tf_circle49h12 = NULL;
apriltag_family_t *tf_custom48h12 = NULL;
apriltag_family_t *tf_standard41h12 = NULL;
apriltag_family_t *tf_standard52h13 = NULL;

// Track which families are enabled
int enabled_families[8] = {0, 0, 0, 0, 0, 0, 0, 0};

// Define tag family types
typedef enum {
    FAMILY_36H11 = 0,
    FAMILY_25H9 = 1,
    FAMILY_16H5 = 2,
    FAMILY_CIRCLE21H7 = 3,
    FAMILY_CIRCLE49H12 = 4,
    FAMILY_CUSTOM48H12 = 5,
    FAMILY_STANDARD41H12 = 6,
    FAMILY_STANDARD52H13 = 7,
    FAMILY_ALL = 8
} tag_family_t;

// Pre-allocate tag family creators
apriltag_family_t* (*tag_creators[])() = {
    tag36h11_create,
    tag25h9_create,
    tag16h5_create,
    tagCircle21h7_create,
    tagCircle49h12_create,
    tagCustom48h12_create,
    tagStandard41h12_create,
    tagStandard52h13_create
};

// Pre-allocate tag family destructors
void (*tag_destructors[])(apriltag_family_t*) = {
    tag36h11_destroy,
    tag25h9_destroy,
    tag16h5_destroy,
    tagCircle21h7_destroy,
    tagCircle49h12_destroy,
    tagCustom48h12_destroy,
    tagStandard41h12_destroy,
    tagStandard52h13_destroy
};

// Get index for tag family
int get_family_index(const char *family_name) {
    if (family_name == NULL) {
        printf("Error: Family name is NULL\n");
        return -1;
    }
    
    if (strcmp(family_name, "tag36h11") == 0) {
        return FAMILY_36H11;
    } else if (strcmp(family_name, "tag25h9") == 0) {
        return FAMILY_25H9;
    } else if (strcmp(family_name, "tag16h5") == 0) {
        return FAMILY_16H5;
    } else if (strcmp(family_name, "tagCircle21h7") == 0) {
        return FAMILY_CIRCLE21H7;
    } else if (strcmp(family_name, "tagCircle49h12") == 0) {
        return FAMILY_CIRCLE49H12;
    } else if (strcmp(family_name, "tagCustom48h12") == 0) {
        return FAMILY_CUSTOM48H12;
    } else if (strcmp(family_name, "tagStandard41h12") == 0) {
        return FAMILY_STANDARD41H12;
    } else if (strcmp(family_name, "tagStandard52h13") == 0) {
        return FAMILY_STANDARD52H13;
    } else if (strcmp(family_name, "tagAll") == 0) {
        return FAMILY_ALL;
    }
    
    printf("Error: Unsupported tag family: '%s'\n", family_name);
    return -1;
}

// Create detector with optimal parameters
void create_detector_if_needed() {
    if (td == NULL) {
        td = apriltag_detector_create();
        td->nthreads = 1;  // Single thread for WebAssembly
        td->refine_edges = 1;
        td->decode_sharpening = 0.25;
        printf("Created new AprilTag detector\n");
    }
}

// Remove and destroy all tag families
void clean_up_tag_families() {
    // Pointers to the tag families
    apriltag_family_t* families[8] = {
        tf_36h11, tf_25h9, tf_16h5, tf_circle21h7, 
        tf_circle49h12, tf_custom48h12, tf_standard41h12, tf_standard52h13
    };
    
    // Remove families from detector and destroy them
    for (int i = 0; i < 8; i++) {
        if (families[i] != NULL && enabled_families[i]) {
            if (td != NULL) {
                apriltag_detector_remove_family(td, families[i]);
            }
            tag_destructors[i](families[i]);
            enabled_families[i] = 0;
        }
    }
    
    // Reset family pointers
    tf_36h11 = NULL;
    tf_25h9 = NULL;
    tf_16h5 = NULL;
    tf_circle21h7 = NULL;
    tf_circle49h12 = NULL;
    tf_custom48h12 = NULL;
    tf_standard41h12 = NULL;
    tf_standard52h13 = NULL;
}

// Add a family to the detector
EMSCRIPTEN_KEEPALIVE
int add_family(int family_idx, int hamming) {
    if (family_idx < 0 || family_idx >= 8) {
        printf("Invalid family index: %d\n", family_idx);
        return -1;
    }
    
    // If this family is already enabled, remove it first
    if (enabled_families[family_idx]) {
        apriltag_family_t* families[8] = {
            tf_36h11, tf_25h9, tf_16h5, tf_circle21h7, 
            tf_circle49h12, tf_custom48h12, tf_standard41h12, tf_standard52h13
        };
        
        if (td != NULL && families[family_idx] != NULL) {
            apriltag_detector_remove_family(td, families[family_idx]);
            tag_destructors[family_idx](families[family_idx]);
            enabled_families[family_idx] = 0;
            
            // Reset the family pointer
            switch (family_idx) {
                case FAMILY_36H11: tf_36h11 = NULL; break;
                case FAMILY_25H9: tf_25h9 = NULL; break;
                case FAMILY_16H5: tf_16h5 = NULL; break;
                case FAMILY_CIRCLE21H7: tf_circle21h7 = NULL; break;
                case FAMILY_CIRCLE49H12: tf_circle49h12 = NULL; break;
                case FAMILY_CUSTOM48H12: tf_custom48h12 = NULL; break;
                case FAMILY_STANDARD41H12: tf_standard41h12 = NULL; break;
                case FAMILY_STANDARD52H13: tf_standard52h13 = NULL; break;
            }
        }
    }
    
    // Special handling for tagStandard52h13 and tagCustom48h12 - enforce minimum hamming distance of 1
    if ((family_idx == FAMILY_STANDARD52H13 || family_idx == FAMILY_CUSTOM48H12) && hamming < 1) {
        printf("Setting minimum hamming distance of 1 for family %d\n", family_idx);
        hamming = 1;
    }
    
    apriltag_family_t* family = tag_creators[family_idx]();
    if (family == NULL) {
        printf("Failed to create tag family %d\n", family_idx);
        return -3;
    }
    
    // Always call add_family_bits() explicitly, even for hamming == 0.
    // apriltag_detector_add_family() (the upstream convenience wrapper this
    // used to call for hamming <= 0) hardcodes bits_corrected=2 regardless
    // of what the caller asked for - harmless for small-code-space families,
    // but for tagCircle49h12 (65535 codes, 49 bits) a hamming-2 decode table
    // needs several GB and calloc() fails outright. add_family_bits() with
    // the exact requested value (0 included) doesn't have that problem.
    apriltag_detector_add_family_bits(td, family, hamming);
    
    // Store the family pointer based on index
    switch (family_idx) {
        case FAMILY_36H11: tf_36h11 = family; break;
        case FAMILY_25H9: tf_25h9 = family; break;
        case FAMILY_16H5: tf_16h5 = family; break;
        case FAMILY_CIRCLE21H7: tf_circle21h7 = family; break;
        case FAMILY_CIRCLE49H12: tf_circle49h12 = family; break;
        case FAMILY_CUSTOM48H12: tf_custom48h12 = family; break;
        case FAMILY_STANDARD41H12: tf_standard41h12 = family; break;
        case FAMILY_STANDARD52H13: tf_standard52h13 = family; break;
    }
    
    enabled_families[family_idx] = 1;
    printf("Added tag family %d with hamming distance %d\n", family_idx, hamming);
    return 0;
}

// Exported function to set the active tag family
EMSCRIPTEN_KEEPALIVE
int initialize_detector(char *family_name, int hamming) {
    printf("Initializing detector with family: '%s', hamming: %d\n", family_name, hamming);

    // Is it a request for all families?
    int is_all_families = (family_name != NULL && strcmp(family_name, "tagAll") == 0);

    // Create detector if it doesn't exist
    create_detector_if_needed();

    // Clean up any existing tag families
    clean_up_tag_families();

    // Get the family index (unless we already know it's all)
    int idx = is_all_families ? FAMILY_ALL : get_family_index(family_name);

    if (idx < 0) {
        return -2; // Unsupported tag family
    }

    // Add specific tag family, honoring the caller's requested hamming
    // distance. This used to hardcode 0 here regardless of what was asked
    // for, silently overriding the first family's own choice - harmless by
    // itself once add_family() stopped substituting hamming 2 for hamming 0
    // (see the comment on that fix above), but still wrong: a caller that
    // explicitly wants hamming 1 for whichever family happens to be added
    // first would silently get hamming 0 instead.
    if (idx < FAMILY_ALL) {
        int result = add_family(idx, hamming);
        if (result != 0) return result;
    }
    // Add all tag families
    else if (idx == FAMILY_ALL) {
        for (int i = 0; i < 8; i++) {
            int result = add_family(i, hamming);
            if (result != 0) return result;
        }
    }

    return 0;
}

// Exported function to set detector parameters
EMSCRIPTEN_KEEPALIVE
void set_detector_parameters(float quad_decimate, float quad_sigma, int refine_edges, float decode_sharpening, int debug) {
    if (td == NULL) {
        printf("Detector not initialized\n");
        return;
    }
    
    td->quad_decimate = quad_decimate;
    td->quad_sigma = quad_sigma;
    td->refine_edges = refine_edges;
    td->decode_sharpening = decode_sharpening;
    td->debug = debug;
    
    printf("Set detector parameters: decimate=%.2f, sigma=%.2f, refine_edges=%d, sharpening=%.2f, debug=%d\n", 
           quad_decimate, quad_sigma, refine_edges, decode_sharpening, debug);
}

// Structure to hold detection results
typedef struct {
    int id;
    int family_id;
    float center_x;
    float center_y;
    float corners[4][2];
    float decision_margin;
    float hamming;
    float rotation;
} detection_info_t;

// Helper function to determine which family a detection belongs to
int get_detection_family_id(apriltag_detection_t *det) {
    // This is a simplistic approach - in reality, you may need a more robust way to identify the family
    // For example, you could add a pointer to the family in the detection struct
    if (tf_36h11 != NULL && det->family == tf_36h11) return FAMILY_36H11;
    if (tf_25h9 != NULL && det->family == tf_25h9) return FAMILY_25H9;
    if (tf_16h5 != NULL && det->family == tf_16h5) return FAMILY_16H5;
    if (tf_circle21h7 != NULL && det->family == tf_circle21h7) return FAMILY_CIRCLE21H7;
    if (tf_circle49h12 != NULL && det->family == tf_circle49h12) return FAMILY_CIRCLE49H12;
    if (tf_custom48h12 != NULL && det->family == tf_custom48h12) return FAMILY_CUSTOM48H12;
    if (tf_standard41h12 != NULL && det->family == tf_standard41h12) return FAMILY_STANDARD41H12;
    if (tf_standard52h13 != NULL && det->family == tf_standard52h13) return FAMILY_STANDARD52H13;
    
    return -1; // Unknown family
}

// Exported function to detect AprilTags in an image
EMSCRIPTEN_KEEPALIVE
int detect_tags(uint8_t *image_data, int width, int height, detection_info_t *detections, int max_detections) {
    if (td == NULL) {
        printf("Detector not initialized\n");
        return -1;
    }
    
    // Create an image_u8 structure from the raw pixel data
    image_u8_t im = {
        .width = width,
        .height = height,
        .stride = width,
        .buf = image_data
    };
    
    // Detect tags in the image
    zarray_t *detections_array = apriltag_detector_detect(td, &im);
    
    // Copy detection results to the output array
    int num_detections = zarray_size(detections_array);
    printf("Detected %d tags\n", num_detections);
    
    int actual_detections = num_detections;
    if (num_detections > max_detections) {
        actual_detections = max_detections;
    }
    
    for (int i = 0; i < actual_detections; i++) {
        // Get detection
        apriltag_detection_t *det;
        zarray_get(detections_array, i, &det);
        
        // Copy detection info
        detections[i].id = det->id;
        detections[i].family_id = get_detection_family_id(det);
        detections[i].center_x = det->c[0];
        detections[i].center_y = det->c[1];
        
        for (int j = 0; j < 4; j++) {
            detections[i].corners[j][0] = det->p[j][0];
            detections[i].corners[j][1] = det->p[j][1];
        }
        
        detections[i].decision_margin = det->decision_margin;
        detections[i].hamming = det->hamming;
        
        // Calculate rotation (average angle of the tag edges)
        double rotation = 0;
        for (int j = 0; j < 4; j++) {
            int next = (j + 1) % 4;
            double dx = det->p[next][0] - det->p[j][0];
            double dy = det->p[next][1] - det->p[j][1];
            rotation += atan2(dy, dx);
        }
        detections[i].rotation = rotation / 4.0;
    }
    
    // Free the detections array
    apriltag_detections_destroy(detections_array);
    
    return actual_detections;
}

// Exported function to clean up resources
EMSCRIPTEN_KEEPALIVE
void cleanup() {
    // Clean up tag families
    clean_up_tag_families();
    
    // Clean up detector
    if (td != NULL) {
        apriltag_detector_destroy(td);
        td = NULL;
    }
    
    printf("Cleanup complete\n");
}