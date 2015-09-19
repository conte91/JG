/** Function that normalizes a vector
 * @param x the x component of the vector
 * @param y the y component of the vector
 * @param z the z component of the vector
 */
template<typename T>
void normalize_vector(T & x, T&y, T&z)
{
  T norm = hypot(hypot(x,y),z);
  x /= norm;
  y /= norm;
  z /= norm;
}
