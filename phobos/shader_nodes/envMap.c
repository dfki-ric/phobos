void envmap_color_frag(in vec2 texCoord, in sampler2D envMapCol, in sampler2D RCol, in sampler2D GCol, in sampler2D BCol, in sampler2D ACol, out vec4 color) {
  vec4 rcol;
  vec4 gcol;
  vec4 bcol;
  vec4 acol;
  vec4 scale;
  rcol = texture2D(RCol, texCoord*envMapScale.r);
  bcol = texture2D(GCol, texCoord*envMapScale.g);
  gcol = texture2D(BCol, texCoord*envMapScale.b);
  acol = texture2D(ACol, texCoord*envMapScale.a);
  scale = texture2D(envMapCol, texCoord);
  color = scale.a * (rcol*scale.r+bcol*scale.b+gcol*scale.g)+acol*(1-scale.a);
}
