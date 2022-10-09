#include "quaternion.h"
#include <math.h>
#include <string.h>

float vector_normalize(struct vector *raw, struct vector *unit)
{
  float mag = sqrt((raw->x) * (raw->x) + (raw->y) * (raw->y) + (raw->z) * (raw->z));
  if(mag != 0) {
      unit->x = raw->x / mag;
      unit->y = raw->y / mag;
      unit->z = raw->z / mag;
  }
  return mag;
}

void vector_add(struct vector *v1, struct vector *v2, struct vector *result)
{
  result->x = v1->x + v2->x;
  result->y = v1->y + v2->y;
  result->z = v1->z + v2->z;
}

void vector_multiply(struct vector *v, float c, struct vector *result)
{
  result->x = v->x * c;
  result->y = v->y * c;
  result->z = v->z * c;
}

void quaternion_create(struct vector *v, float angle, struct quaternion *result)
{
  result->r = cos(angle/2);
  result->i = v->x * sin(angle/2);
  result->j = v->y * sin(angle/2);
  result->k = v->z * sin(angle/2);
}

void quaternion_rotate(struct vector *v, struct quaternion *q, struct vector *result)
{
  struct vector n;
  memcpy(&n, v, sizeof(n));
  result->x = n.x * (1 - 2*((q->j)*(q->j)) + (q->k)*(q->k)) + n.y * 2*(q->i * q->j - q->k * q->r) + n.z * 2*(q->i * q->k + q->j * q->r);
  result->y = n.x * 2*(q->i * q->j + q->k * q->r) + n.y * (1 - 2*((q->i)*(q->i) + (q->k)*(q->k))) + n.z * 2*(q->j * q->k - q->i * q->r);
  result->z = n.x * 2*(q->i * q->k - q->j * q->r) + n.y * 2*(q->j * q->k + q->i * q->r) + n.z * (1 - 2*((q->i)*(q->i) + (q->j)*(q->j)));
}

void quaternion_multiply(struct quaternion *q1, struct quaternion *q2, struct quaternion *result)
{
  result->r = q1->r * q2->r - q1->i * q2->i - q1->j * q2->j - q1->k * q2->k;
  result->i = q1->r * q2->i + q1->i * q2->r + q1->j * q2->k - q1->k * q2->j;
  result->j = q1->r * q2->j - q1->i * q2->k + q1->j * q2->r + q1->k * q2->i;
  result->k = q1->r * q2->k + q1->i * q2->j - q1->j * q2->i + q1->k * q2->r;
}

float vector_roll(struct vector *v)
{
  return atan2(v->x, sqrt((v->z) * (v->z) + (v->y) * (v->y)));
}

float vector_pitch(struct vector *v)
{
  return -atan2(v->y, v->z);
}
