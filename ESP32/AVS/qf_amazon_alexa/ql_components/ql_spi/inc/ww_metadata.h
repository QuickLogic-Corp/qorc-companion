#ifndef __WW_METADATA_H__ 
#define __WW_METADATA_H__

#define MAX_KEPHRASE_SCORES (16)
typedef struct t_ql_audio_meta_data_
{
  int32_t n_keyphrase_count; // number of key phrases supported - default is 0
  int32_t n_keyphrase_triggered_index; // Index in the array of keyphrases
  int32_t n_keyphrase_start_index; // Start of Key Phrase in number of samples from start of buffer
  int32_t n_keyphrase_end_index;  // End of Key Phrase in number of samples from start of buffer 
  int32_t a_kephrase_score;
  int32_t n_rdsp_length_estimate; // RDSP WW Length estimate 
} t_ql_audio_meta_data;

#endif /* WW_METADATA_H */